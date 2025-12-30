import logging
import os
import sys
import unittest
from unittest.mock import MagicMock, mock_open, patch

# Add src to sys.path so we can import rostrace
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))

from rostrace.runner import Runner

# Disable logging for tests to keep output clean
logging.getLogger("rostrace").addHandler(logging.NullHandler())
logging.getLogger("rostrace").propagate = False


class TestRunner(unittest.TestCase):
    def setUp(self):
        # Mock args
        self.args = MagicMock()
        self.args.pid = None
        self.args.node = None
        self.args.lib = None
        self.runner = Runner(self.args)

    @patch("os.geteuid")
    def test_check_root_privileges(self, mock_geteuid):
        """Test root privilege check."""
        mock_geteuid.return_value = 0
        self.assertTrue(Runner.check_root_privileges())

        mock_geteuid.return_value = 1000
        self.assertTrue(Runner.check_root_privileges())

    @patch("os.path.exists")
    def test_check_debug_fs_mount(self, mock_exists):
        """Test DebugFS mount check."""
        # Case 1: Exists
        mock_exists.return_value = True
        self.assertTrue(Runner.check_debug_fs_mount())

        # Case 2: Missing
        mock_exists.return_value = False
        self.assertFalse(Runner.check_debug_fs_mount())

    def test_check_preflight(self):
        """Test that check_preflight calls the individual checks."""
        # Mock the static methods on the class
        with patch.object(
            Runner, "check_root_privileges", return_value=True
        ) as mock_root, patch.object(
            Runner, "check_debug_fs_mount", return_value=True
        ) as mock_debug:
            self.assertTrue(self.runner.check_preflight())

            # Verify calls
            mock_root.assert_called_once()
            mock_debug.assert_called_once()

        # Test failure case
        with patch.object(
            Runner, "check_root_privileges", return_value=True
        ), patch.object(Runner, "check_debug_fs_mount", return_value=False):
            self.assertFalse(self.runner.check_preflight())

    @patch("builtins.open", new_callable=mock_open, read_data="NSpid:\t256123\t170\n")
    def test_get_host_pid(self, mock_file):
        """Test parsing NSpid from status file."""
        host_pid = self.runner.get_host_pid(170)
        self.assertEqual(host_pid, 256123)

    @patch("subprocess.check_output")
    def test_resolve_pid_success(self, mock_check_output):
        """Test that resolve_pid correctly extracts PID from rosnode info output."""
        mock_output = b"\nNode [/talker]\nPid: 12345\n"
        mock_check_output.return_value = mock_output
        pid = self.runner.resolve_pid("/talker")
        self.assertEqual(pid, 12345)

    @patch("subprocess.check_output")
    @patch("os.geteuid")
    def test_resolve_symbol(self, mock_geteuid, mock_check_output):
        """Test dynamic symbol resolution logic."""
        mock_geteuid.return_value = 0

        # Scenario 1: Multiple symbols, pick the one with 'Ev'
        mock_check_output.return_value = b"""
uprobe:/lib/lib.so:_Z3foov
uprobe:/lib/lib.so:_Z3fooEv
uprobe:/lib/lib.so:_Z3fooi
"""
        sym = self.runner.resolve_symbol("/lib/lib.so", "foo")
        self.assertEqual(sym, "_Z3fooEv")

        # Scenario 2: No 'Ev', pick the shortest
        mock_check_output.return_value = b"""
uprobe:/lib/lib.so:_Z3barLongName
uprobe:/lib/lib.so:_Z3barShort
"""
        sym = self.runner.resolve_symbol("/lib/lib.so", "bar")
        self.assertEqual(sym, "_Z3barShort")

    @patch("subprocess.check_output")
    @patch("os.geteuid")
    @patch("os.remove")
    @patch("tempfile.NamedTemporaryFile")
    @patch("subprocess.call")
    @patch("builtins.open", new_callable=mock_open)
    @patch("os.path.exists")
    def test_run_logic_flow(
        self,
        mock_exists,
        mock_file,
        mock_call,
        mock_temp,
        mock_remove,
        mock_geteuid,
        mock_check_output,
    ):
        """Test full run orchestration."""
        self.args.pid = 170

        # Mock not being root (so we expect sudo)
        mock_geteuid.return_value = 1000

        # Mock pre-flight and file checks
        mock_exists.return_value = True

        # Mock file reading
        def open_side_effect(filename, mode="r"):
            if filename.endswith("status"):
                return mock_open(
                    read_data="Name:\tprocess\nNSpid:\t256123\t170\n"
                ).return_value
            elif filename.endswith("maps"):
                return mock_open(
                    read_data="7f8e... r-xp ... /opt/ros/noetic/lib/libroscpp.so\n"
                ).return_value
            elif filename.endswith("topics.bt"):
                return mock_open(
                    read_data="uprobe:{{LIBROSCPP_PATH}}:{{SYMBOL_CALL_ONE_CB}}:"
                    "{{SYMBOL_HANDLE_MESSAGE}}:{{SYMBOL_ADD_CALLBACK}}"
                ).return_value
            return mock_open().return_value

        mock_file.side_effect = open_side_effect

        # Mock symbol resolution (check_output)
        mock_check_output.return_value = (
            b"uprobe:/lib/lib.so:_ZN3ros13CallbackQueue9callOneCBEv"
        )

        # Mock temp file
        mock_temp_obj = MagicMock()
        mock_temp_obj.name = "/tmp/test.bt"
        mock_temp.return_value.__enter__.return_value = mock_temp_obj

        # Execute
        self.runner.run()

        # Verify host PID was resolved
        mock_call.assert_called_with(["sudo", "bpftrace", "/tmp/test.bt", "256123"])

        # Verify symbols were replaced
        handle = mock_temp_obj.write
        sym = "_ZN3ros13CallbackQueue9callOneCBEv"
        expected_content = f"uprobe:/opt/ros/noetic/lib/libroscpp.so:{sym}:{sym}:{sym}"
        handle.assert_called_with(expected_content)


if __name__ == "__main__":
    unittest.main()
