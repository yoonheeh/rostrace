import logging
import os
import subprocess
import tempfile
from typing import Any, Optional

# Set up logging
logger = logging.getLogger("rostrace")


class Runner:
    def __init__(self, args: Any):
        self.args = args
        # Use relative path to find BPF scripts, working for both source and install
        self.bpf_dir = os.path.join(os.path.dirname(__file__), "bpf")

    @staticmethod
    def check_root_privileges() -> bool:
        if os.geteuid() != 0:
            logger.warning(
                "Not running as root. rostrace will attempt to use 'sudo' "
                "for kernel access."
            )
        return True

    @staticmethod
    def check_debug_fs_mount() -> bool:
        debugfs_path = "/sys/kernel/debug/tracing"
        if not os.path.exists(debugfs_path):
            logger.error("Kernel debug filesystem not found at %s", debugfs_path)
            logger.info(
                "  If running in Docker, ensure you added: "
                "-v /sys/kernel/debug:/sys/kernel/debug:rw"
            )
            logger.info("  and use --privileged.")
            return False
        return True

    def check_preflight(self) -> bool:
        """
        Checks if the environment is ready for eBPF tracing.
        """
        return self.check_root_privileges() and self.check_debug_fs_mount()

    def resolve_pid(self, node_name: str) -> Optional[int]:
        """
        Resolves a ROS node name to a Process ID (PID).
        """
        try:
            logger.debug("Resolving PID for node: %s", node_name)
            output = subprocess.check_output(
                ["rosnode", "info", node_name], stderr=subprocess.STDOUT
            ).decode()

            for line in output.split("\n"):
                if line.strip().startswith("Pid:"):
                    return int(line.split(":")[1].strip())
        except subprocess.CalledProcessError as e:
            logger.error(
                "Failed to resolve node PID: 'rosnode info' failed for %s", node_name
            )
            logger.debug("rosnode output: %s", e.output.decode().strip())
            return None
        except (ValueError, IndexError) as e:
            logger.error(
                "Malformed output from 'rosnode info' while resolving PID: %s", e
            )
            return None
        except Exception as e:
            logger.error("Unexpected error resolving node PID: %s", e)
            return None

        logger.error("Node '%s' found but no PID reported by ROS Master.", node_name)
        return None

    def get_host_pid(self, container_pid: int) -> int:
        """
        Reads /proc/<pid>/status to find the NSpid line.
        Returns the host PID (the first PID in the list).
        """
        status_path = f"/proc/{container_pid}/status"
        try:
            with open(status_path, "r") as f:
                for line in f:
                    if line.startswith("NSpid:"):
                        # Format: NSpid:	256123	170
                        parts = line.split()
                        if len(parts) >= 2:
                            return int(parts[1])
        except FileNotFoundError:
            logger.error(
                "Process status not found: %s. Is the node still running?", status_path
            )
        except Exception as e:
            logger.warning("Could not read NSpid from %s: %s", status_path, e)

        return container_pid

    def get_libroscpp_path(self, pid: int) -> Optional[str]:
        """
        Scans /proc/PID/maps to find the location of libroscpp.so.
        This handles both native and containerized processes (relative to the host).
        """
        maps_path = f"/proc/{pid}/maps"
        logger.debug("Scanning maps file: %s", maps_path)
        found_in_maps = False
        try:
            with open(maps_path, "r") as f:
                for line in f:
                    if "libroscpp.so" in line:
                        found_in_maps = True
                        parts = line.split()
                        if len(parts) >= 6:
                            path = parts[5]
                            # Handle container perspective
                            full_path = f"/proc/{pid}/root{path}"

                            logger.debug(
                                "Checking map entry: '%s' -> Host path: '%s'",
                                path,
                                full_path,
                            )

                            if os.path.exists(full_path):
                                logger.debug(
                                    "Found accessible path via /proc: %s", full_path
                                )
                                return full_path
                            elif os.path.exists(path):
                                logger.debug("Found accessible path locally: %s", path)
                                return path
                            else:
                                logger.debug(
                                    "Path not accessible: %s AND %s", full_path, path
                                )
        except PermissionError:
            logger.error(
                "Permission denied reading %s. Try running with root/sudo.", maps_path
            )
        except Exception as e:
            logger.error("Error scanning process maps at %s: %s", maps_path, e)

        if found_in_maps:
            logger.warning(
                "Found 'libroscpp.so' in maps, but could not access the file on disk."
            )
        else:
            logger.warning("No 'libroscpp.so' entry found in %s", maps_path)

        return None

    def resolve_symbol(self, lib_path: str, pattern: str) -> Optional[str]:
        """
        Uses bpftrace -l to find the exact mangled symbol name.
        Applies heuristics to pick the best match.
        """
        query = f"uprobe:{lib_path}:{pattern}"
        cmd = ["bpftrace", "-l", query]
        if os.geteuid() != 0:
            cmd = ["sudo"] + cmd

        try:
            output = subprocess.check_output(cmd, stderr=subprocess.PIPE).decode()
            candidates = [
                line.strip().split(":")[-1]
                for line in output.split("\n")
                if line.strip()
            ]
        except subprocess.CalledProcessError:
            return None

        if not candidates:
            return None

        # Heuristic 1: Prefer symbols ending in 'Ev' (standard void entry point)
        # KNOWN LIMITATION: This assumes Itanium C++ ABI.
        for c in candidates:
            if c.endswith("Ev"):
                return c

        # Heuristic 2: Shortest symbol (likely the base function)
        # KNOWN LIMITATION: May hook overloads if the void signature is missing.
        return min(candidates, key=len)

    def is_python_process(self, pid: int) -> bool:
        """
        Checks if the process is a Python interpreter.
        """
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                # cmdline arguments are null-separated
                cmdline = f.read().decode(errors="ignore").replace("\0", " ")
            if "python" in cmdline or "rospy" in cmdline:
                return True
        except Exception as e:
            logger.debug("Could not check process cmdline: %s", e)
        return False

    def resolve_exe_path(self, pid: int) -> Optional[str]:
        """
        Resolves the executable path from /proc/PID/exe.
        Handles container paths relative to host.
        """
        try:
            exe_link = f"/proc/{pid}/exe"
            if os.path.exists(exe_link):
                # readlink gives the path inside the container/namespace
                container_path = os.readlink(exe_link)
                # Convert to host path
                host_path = f"/proc/{pid}/root{container_path}"
                if os.path.exists(host_path):
                    return host_path
                # Fallback: maybe it's accessible directly (host process)
                if os.path.exists(container_path):
                    return container_path
        except Exception as e:
            logger.debug("Failed to resolve exe path for PID %d: %s", pid, e)
        return None

    def run(self) -> None:
        if not self.check_preflight():
            return

        target_pid = self.args.pid
        if not target_pid and self.args.node:
            target_pid = self.resolve_pid(self.args.node)

        if not target_pid:
            logger.error(
                "Target PID not provided and could not be resolved from node name."
            )
            return

        # Resolve the Host PID for BPF tracing
        host_pid = self.get_host_pid(target_pid)
        logger.info("Tracing Node PID: %d (Host PID: %d)", target_pid, host_pid)

        # 1. Find libroscpp.so
        if self.args.lib:
            lib_path = self.args.lib
            if not os.path.isfile(lib_path):
                logger.error(
                    "User-specified library path does not exist or is not a file: %s",
                    lib_path,
                )
                return
            logger.info("Using user-specified libroscpp.so at: %s", lib_path)
        else:
            lib_path = self.get_libroscpp_path(target_pid)
            if not lib_path:
                # Check for common incompatibility causes
                if self.is_python_process(target_pid):
                    logger.error(
                        "Target PID %d appears to be a Python process.", target_pid
                    )
                    logger.error("rostrace currently ONLY supports C++ (roscpp) nodes.")
                    logger.error(
                        "Python nodes (rospy) use a different middleware "
                        "implementation not yet supported."
                    )
                    return

                # Check if symbols are statically linked in the main executable
                # (e.g., Bazel)
                exe_path = self.resolve_exe_path(target_pid)
                if exe_path:
                    logger.debug(
                        "Checking for static symbols in executable: %s", exe_path
                    )
                    # Use a characteristic symbol to test
                    if self.resolve_symbol(exe_path, "*Subscription*handleMessage*"):
                        logger.info(
                            "libroscpp.so not found, but symbols detected in "
                            "executable: %s",
                            exe_path,
                        )
                        lib_path = exe_path

                if not lib_path:
                    logger.warning(
                        "libroscpp.so not found in maps and no static symbols detected."
                    )
                    logger.warning(
                        "Falling back to default system library: "
                        "/opt/ros/noetic/lib/libroscpp.so"
                    )
                    logger.warning(
                        "If the trace is empty, try pointing --lib to your node's "
                        "executable."
                    )
                    lib_path = "/opt/ros/noetic/lib/libroscpp.so"
            else:
                logger.info("Auto-detected libroscpp.so at: %s", lib_path)

        # 2. Resolve Symbols Dynamically
        logger.info("Resolving symbols in %s...", lib_path)
        sym_handle = self.resolve_symbol(lib_path, "*Subscription*handleMessage*")
        sym_add = self.resolve_symbol(lib_path, "*CallbackQueue*addCallback*")
        sym_call = self.resolve_symbol(lib_path, "*CallbackQueue*callOneCB*")

        if not (sym_handle and sym_add and sym_call):
            logger.error("Could not resolve one or more symbols in libroscpp.so")
            logger.info("  handleMessage: %s", sym_handle or "NOT FOUND")
            logger.info("  addCallback:   %s", sym_add or "NOT FOUND")
            logger.info("  callOneCB:     %s", sym_call or "NOT FOUND")
            return

        logger.debug("Selected Symbols:")
        logger.debug("  handleMessage -> %s", sym_handle)
        logger.debug("  addCallback   -> %s", sym_add)
        logger.debug("  callOneCB     -> %s", sym_call)

        # 3. Prepare BPF script
        script_path = os.path.join(self.bpf_dir, "topics.bt")
        if not os.path.exists(script_path):
            logger.error("BPF script template not found at %s", script_path)
            return

        try:
            with open(script_path, "r") as f:
                script_content = f.read()

            script_content = script_content.replace("{{LIBROSCPP_PATH}}", lib_path)
            script_content = script_content.replace(
                "{{SYMBOL_HANDLE_MESSAGE}}", sym_handle
            )
            script_content = script_content.replace("{{SYMBOL_ADD_CALLBACK}}", sym_add)
            script_content = script_content.replace("{{SYMBOL_CALL_ONE_CB}}", sym_call)

            with tempfile.NamedTemporaryFile(
                mode="w", suffix=".bt", delete=False
            ) as temp_script:
                temp_script.write(script_content)
                temp_script_path = temp_script.name
        except Exception as e:
            logger.error("Failed to generate BPF script: %s", e)
            return

        # 4. Build the command
        prefix = ["sudo"] if os.geteuid() != 0 else []
        cmd = prefix + ["bpftrace", temp_script_path, str(host_pid)]

        logger.info("Starting bpftrace... (Ctrl-C to stop and show results)")
        logger.debug("Command: %s", " ".join(cmd))

        try:
            subprocess.call(cmd)
        except KeyboardInterrupt:
            # Expected on exit
            pass
        except Exception as e:
            logger.error("bpftrace execution failed: %s", e)
        finally:
            if os.path.exists(temp_script_path):
                os.remove(temp_script_path)
