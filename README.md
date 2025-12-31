# rostrace
An eBPF-based performance tool for pinpointing latency bottlenecks in
multi-process ROS systems.

## Quick Start

The fastest way to see `rostrace` in action is using our Docker-based demos:

```bash
# 1. Start the demo environment (Terminal 1)
docker compose -f demo/docker-compose.processing.yml up --build

# 2. Run the trace tool (Terminal 2)
./rostrace-docker trace --node /processing_delay_node --topic /slow_topic
```

For detailed instructions on running native nodes on your host, advanced configuration, and troubleshooting, see the [**Usage Guide**](Usage.md).

## The Problem
You publish a topic with a desired frequency - let's say 20Hz. You know it has
to arrive every 50ms, but it keeps missing its schedule.
Now you want to know: why?
Standard ROS tools (like `rostopic hz` or `rqt_graph`) treat the nodes as a
black box. They can tell you a message arrived late, but they cannot tell you
why.
 - Was your code inefficient?
 - Was the OS scheduler starving the thread?
 - Did a "silent" page fault pause the CPU?
 - Was the message dropped inside the middleware queue before the callback
   even started?

## The Solution
`rostrace` uses eBPF (via `bpftrace`) to peek inside the "black box" of both
ROS middleware and the Linux kernel simultaneously. It decomposes the
lifecycle of a message into four distinct metrics:

*   **Network Latency** (Planned)
*   **Serialization Latency** (Implemented)
*   **Queuing Latency** (Implemented)
*   **Processing Latency** (Implemented)

## Why eBPF?

To debug a robot, you must see through the layers of the Operating System.

*   **Multi-Process (Distributed) Debugging**: ROS splits a robot's brain into
    many independent processes. Latency often hides in the communication
    between these processes. `rostrace` tracks messages as they cross
    process boundaries.
*   **Zero-Instrumentation**: `rostrace` attaches to running processes
    dynamically. You don't need to change a single line of your code or
    recompile your nodes.
*   **Eliminating Heisenbugs**: Standard tools run in *User-Space* and are
    biased by the very latency they try to measure. `rostrace` runs inside
    the *Linux Kernel*. It captures 100% of events with nanosecond precision,
    seeing system-wide events—like the OS scheduler preempting your node—that
    are invisible to ROS.

### Tracing the Entire ROS Ecosystem

While ROS offers many communication abstractions (Topics, Services, Actions, TF),
they all ultimately rely on common middleware patterns: data serialization,
internal queuing, and callback dispatch.

`rostrace` targets these fundamental building blocks inside `libroscpp.so`,
allowing it to provide latency insights across the entire ROS graph using a
unified analysis approach.

## Performance Bottleneck Analysis

`rostrace` breaks down the fault domain by measuring how long a message
spends in each internal stage:

*   **Network Latency (Planned)**: From kernel TCP stack entry until the
    middleware begins processing the raw buffer.
*   **Serialization Latency**: Measured from the start to the end of the
    message deserialization process. This identifies overhead from large or
    complex message types.
*   **Queuing Latency**: The time a message sits "ready and waiting" in the
    `ros::CallbackQueue` before your code is free to process it.
*   **Processing Latency**: The time your own callback code took to execute.

### How to Interpret the Results

*   **High Serialization Latency**: Indicates the message format is a
    bottleneck. Consider optimizing message structures or using compression.
*   **High Queuing Latency**: Indicates the node is overloaded or being
    starved of CPU time by the OS scheduler.
*   **High Processing Latency**: Indicates an inefficiency inside your
    callback function (e.g., heavy algorithms or blocking I/O).

## Technical Architecture

*   **Event Tracing**: Unlike statistical profilers (like `parca`) that sample
    the CPU at intervals, `rostrace` uses deterministic event tracing. It
    captures 100% of occurrences, ensuring that even a single 100ms latency
    spike is never missed.
*   **Dynamic Symbol Resolution**: The tool automatically discovers mangled
    C++ symbols in your specific version of `libroscpp.so`, ensuring robust
    operation across different ROS distributions.
*   **Container Compatibility**: Includes specific logic to resolve
    namespaced PIDs, allowing you to trace production containers from the
    host or within a privileged sidecar.

## Future Work

### Extended Metrics
*   **Network Latency (Kernel-to-User)**: Tracing from `tcp_recvmsg` to
    middleware entry using `kprobes`.
*   **Action-Specific Latency**: Direct tracing of the `actionlib` API to
    measure **Goal-to-Accept** and **Feedback** delay.
*   **TF Lookup Latency**: Measuring time spent inside blocking
    `lookupTransform()` calls.

### Root Cause Analysis (System Events)
*   **Scheduler Starvation**: Correlating latency with `sched_switch` events
    to detect when a node is blocked by other processes.
*   **Context Switch Storms**: Detecting thrashing due to high-frequency
    preemption or priority inversion.

### Reliability & Failure Detection
*   **Black-Box Drop Detection**: Identifying packet loss by tracking
    divergence between arrival rates and execution rates.
*   **Real-Time Safety Auditing**: Flagging dynamic memory allocation
    (`malloc`) or blocking I/O inside critical control loops.
