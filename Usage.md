# rostrace Usage Guide

This guide explains how to use `rostrace` to identify and debug latency in your ROS 1 systems.

## 1. Quick Start with Demos (Docker)

We provide pre-packaged demos using Docker Compose. These scenarios simulate common latency issues without requiring any installation on your host machine.

### Prerequisites
- Docker and Docker Compose
- Linux kernel with eBPF support (Standard on Ubuntu 20.04+)

### Scenario A: Processing Latency
In this scenario, a node has a callback that takes ~200ms to complete.

1.  **Start the environment**:
    ```bash
    docker compose -f demo/docker-compose.processing.yml up --build
    ```
2.  **Run the trace**:
    Open a new terminal and enter the tool container:
    ```bash
    docker exec -it rostrace_tool_proc bash
    rostrace trace --node /processing_delay_node --topic /slow_topic
    ```

### Scenario B: Queuing Latency (Service Blocking)
In this scenario, a node is busy handling a heavy service call, causing incoming topic messages to wait in the queue.

1.  **Start the environment**:
    ```bash
    docker compose -f demo/docker-compose.queuing.yml up --build
    ```
2.  **Run the trace**:
    Open a new terminal and enter the tool container:
    ```bash
    docker exec -it rostrace_tool_queue bash
    rostrace trace --node /service_blocker_node --topic /blocked_topic
    ```

---

## 2. Running on Host (Native)

To trace your own native ROS nodes running directly on your machine:

### Installation
1.  **Install System Dependencies**:
    ```bash
    sudo apt-get install bpftrace
    ```
2.  **Build the ROS Package**:
    Place the `rostrace` folder in your catkin workspace (e.g., `~/catkin_ws/src/`) and build:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

### Execution
Run the tool using `rosrun`:
```bash
rosrun rostrace rostrace trace --node /my_node_name --topic /my_topic_name
```

---

## 3. Advanced Configuration

### Explicit Library Path (`--lib`)
If `rostrace` cannot automatically find `libroscpp.so` (common in custom builds or statically linked environments), you can provide the path explicitly:

```bash
rostrace trace --node /my_node --lib /opt/custom_ros/lib/libroscpp.so
```

### Virtual Environments
If you prefer to run the Python logic in a virtual environment:
1.  Create and activate your venv.
2.  Install dependencies: `pip install .` (from the project root).
3.  Note: You still need `bpftrace` installed on your system.

### Troubleshooting
- **Missing DebugFS**: If you see "Kernel debug filesystem not found", ensure `/sys/kernel/debug` is mounted. On host: `sudo mount -t debugfs none /sys/kernel/debug`.
- **Permissions**: `rostrace` requires root privileges to attach eBPF probes. It will attempt to use `sudo` automatically if not run as root.
