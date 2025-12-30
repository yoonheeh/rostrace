# Dockerfile for creating a ROS1 Noetic demo environment for rostrace
FROM ros:noetic-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    bpftrace \
    python3 \
    python3-pip \
    procps \
    kmod \
    && rm -rf /var/lib/apt/lists/*

# Create a catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Copy project
COPY . /catkin_ws/src/rostrace

# Organize packages
RUN mv /catkin_ws/src/rostrace/example/queuing_latency /catkin_ws/src/queuing_latency
RUN mv /catkin_ws/src/rostrace/example/scenarios /catkin_ws/src/scenarios

# Build
WORKDIR /catkin_ws
RUN apt-get update && \
    (rosdep init || true) && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace (rostrace tool + queuing_latency example)
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_make"

# Install rostrace as a python package so it is available globally
RUN pip3 install /catkin_ws/src/rostrace

# No ENTRYPOINT - let the user or Compose define the command
CMD ["bash"]
