# Example: Diagnosing High Queuing Latency

This example demonstrates how to use `rosbpf` to diagnose high queuing
latency caused by CPU starvation.

## 1. Build and Run the Example

This is a standard ROS package. To build it, place this directory in a
catkin workspace, and run `catkin_make`.

```bash
# In your catkin workspace
catkin_make
source devel/setup.bash
```

Now, run the nodes. You will need three separate terminals.

**Terminal 1: Start `roscore`**
```bash
roscore
```

**Terminal 2: Run the `listener` node**
```bash
rosrun queuing_latency_example listener
```

**Terminal 3: Run the `talker` node**
```bash
rosrun queuing_latency_example talker
```

At this point, the `talker` is publishing messages at 20Hz, and the
`listener` is receiving them.

## 2. Profile with `rosbpf` (Healthy State)

Let's see a baseline of the system's performance.

First, find the Process ID (PID) of your `listener` node:
```bash
pgrep -f listener
# Example output: 12345
```

Now, run `rosbpf` on that PID, targeting the `my_topic` topic.
```bash
# Make sure you have root/sudo access to run eBPF tools
sudo rosbpf-latency-analysis --pid 12345 --topic my_topic
```

In a healthy system, the output should show low queuing latency (~2ms):

```
>>> Latency for topic 'my_topic' (ns)
                transport      queuing    processing
count           100.00000    100.000000    100.000000
mean         500000.00000  2000000.00000  10000000.0000
std           50000.00000   200000.00000    100000.00000
min          450000.00000  1800000.00000   9900000.00000
max          550000.00000  2200000.00000  10100000.00000
```

## 3. Simulate CPU Starvation

Now, let's simulate an overloaded CPU. We provide a simple script to start
a process that will consume 100% of one CPU core.

**Terminal 4: Start the CPU stress**
```bash
./start_cpu_stress.sh
```

With this process running, our `listener` node now has to compete for CPU
time.

## 4. Diagnose the Issue

Run `rosbpf` again while the CPU is under load.

```bash
sudo rosbpf-latency-analysis --pid 12345 --topic my_topic
```

This time, the results tell a very different story. The queuing latency will
be significantly higher, confirming that messages are stuck waiting in the
subscription queue.

```
>>> Latency for topic 'my_topic' (ns)
                transport      queuing    processing
count           100.00000    100.000000    100.000000
mean         500000.00000  45000000.0000  10000000.0000
std           50000.00000  15000000.0000    100000.00000
min          450000.00000  25000000.0000   9900000.00000
max          550000.00000  65000000.0000  10100000.00000
```
This result is a smoking gun: messages are arriving at the node just fine
(low transport latency), but they are waiting a long time before the
callback can be run.

## 5. Stop the Simulation and Verify

Finally, stop the CPU stressor.

**Terminal 4: Stop the CPU stress**
```bash
./stop_cpu_stress.sh
```

If you run `rosbpf` again, you will see the queuing latency return to its
normal, healthy state. This confirms that the performance problem was
caused by CPU starvation.
