# rostrace Usage Guide

This guide explains how to use `rostrace` to identify and debug latency in your ROS 1 systems.

## 1. Quick Start with Demos (Docker)

We provide pre-packaged demos using Docker Compose. These scenarios simulate common 
latency issues without requiring any installation on your host machine.

### Prerequisites
- Docker and Docker Compose
- Linux kernel with eBPF support (Standard on Ubuntu 20.04+)

### 1.1 The Streamlined Way (Recommended)

To make tracing as seamless as possible, we provide a wrapper script 
`./rostrace-docker` in the project root. This script automatically handles all 
the complex Docker flags required for eBPF tracing.

```bash
# General usage
./rostrace-docker trace --node /YOUR_NODE --topic /YOUR_TOPIC
```

### Scenario A: Processing Latency
In this scenario, a node has a callback that takes ~200ms to complete.

1.  **Start the environment**:
    ```bash
    docker compose -f demo/docker-compose.processing.yml up --build
    ```
2.  **Run the trace**:
    Open a new terminal and run the wrapper:
    ```bash
    ./rostrace-docker trace --node /processing_delay_node --topic /slow_topic
    ```
You will see an output like below:
```
1. Serialization Latency (us):
@serialization_latency_us: 
[2, 4)                 1 |@                                                   |
[4, 8)                50 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|
[8, 16)                5 |@@@@@                                               |
[16, 32)               2 |@@                                                  |



2. Queuing Latency (us):
@queuing_latency_us: 
[128K, 256K)          57 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|



3. Processing Latency (us):
@processing_latency_us: 
[128K, 256K)          56 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|
```

You can see that due to **High Processing Latency** (~200ms), the node cannot 
keep up with incoming messages. This causes them to back up, leading to a direct 
increase in **Queuing Latency** (also ~200ms). The serialization overhead remains negligible.

### Scenario B: Queuing Latency (Service Blocking)
In this scenario, a node is busy handling a heavy service call, causing incoming 
topic messages to wait in the queue.

1.  **Start the environment**:
    ```bash
    docker compose -f demo/docker-compose.queuing.yml up --build
    ```
2.  **Run the trace**:
    Open a new terminal and run the wrapper:
    ```bash
    ./rostrace-docker trace --node /service_blocker_node --topic /blocked_topic
    ```
You will see an output like below:
```
1. Serialization Latency (us):
@serialization_latency_us: 
[4, 8)               128 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|
[8, 16)               19 |@@@@@@@                                             |
[16, 32)              33 |@@@@@@@@@@@@@                                       |
[32, 64)               2 |                                                    |



2. Queuing Latency (us):
@queuing_latency_us: 
[8, 16)                9 |@@@@@@                                              |
[16, 32)              31 |@@@@@@@@@@@@@@@@@@@@@@@@                            |
[32, 64)              67 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|
[64, 128)              4 |@@@                                                 |
[128, 256)             0 |                                                    |
[256, 512)             0 |                                                    |
[512, 1K)              0 |                                                    |
[1K, 2K)               0 |                                                    |
[2K, 4K)               2 |@                                                   |
[4K, 8K)              16 |@@@@@@@@@@@@                                        |
[8K, 16K)              0 |                                                    |
[16K, 32K)             0 |                                                    |
[32K, 64K)             0 |                                                    |
[64K, 128K)           18 |@@@@@@@@@@@@@                                       |
[128K, 256K)          18 |@@@@@@@@@@@@@                                       |
[256K, 512K)          36 |@@@@@@@@@@@@@@@@@@@@@@@@@@@                         |



3. Processing Latency (us):
@processing_latency_us: 
[8, 16)               28 |@@@@@@@@@@@@@@                                      |
[16, 32)              29 |@@@@@@@@@@@@@@@                                     |
[32, 64)              99 |@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@|
[64, 128)             24 |@@@@@@@@@@@@                                        |
[128, 256)             1 |                                                    |
[256, 512)             0 |                                                    |
[512, 1K)              0 |                                                    |
[1K, 2K)               0 |                                                    |
[2K, 4K)               0 |                                                    |
[4K, 8K)               0 |                                                    |
[8K, 16K)              0 |                                                    |
[16K, 32K)             0 |                                                    |
[32K, 64K)             0 |                                                    |
[64K, 128K)            0 |                                                    |
[128K, 256K)           0 |                                                    |
[256K, 512K)          18 |@@@@@@@@@                                           |

```

This scenario reveals a **bimodal distribution** in Processing Latency. The 
cluster in the microseconds range represents your fast topic callbacks. The 
cluster at ~500ms represents the `heavy_service` handler blocking the main thread. 

Note how the **Queuing Latency** shows a wide spread: messages that arrive while 
the service is running are forced to wait, while messages that arrive when the 
thread is free are processed instantly. This is a classic "Thread Starvation" pattern.


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

## 3. Advanced Configuration

### Explicit Library Path (`--lib`)
If `rostrace` cannot automatically find `libroscpp.so` (common in custom builds 
or statically linked environments), you can provide the path explicitly:

```bash
rostrace trace --node /my_node --lib /opt/custom_ros/lib/libroscpp.so
```

### Virtual Environments
If you prefer to run the Python logic in a virtual environment:
1.  Create and activate your venv.
2.  Install dependencies: `pip install .` (from the project root).
3.  Note: You still need `bpftrace` installed on your system.

### Troubleshooting
- **Missing DebugFS**: If you see "Kernel debug filesystem not found", ensure 
`/sys/kernel/debug` is mounted. On host: `sudo mount -t debugfs none /sys/kernel/debug`.
- **Permissions**: `rostrace` requires root privileges to attach eBPF probes. 
It will attempt to use `sudo` automatically if not run as root.
