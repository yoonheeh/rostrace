# Dockerfile for creating a ROS1 Noetic demo environment
# This image includes the rostrace tool AND compiled C++ examples (scenarios)
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

# Copy the entire project context (including examples)
# Note: The build context must be the repository root
COPY . /catkin_ws/src/rostrace

# Organize packages for compilation
# We move the example packages to the top level of the workspace
RUN mv /catkin_ws/src/rostrace/example/queuing_latency /catkin_ws/src/queuing_latency
RUN mv /catkin_ws/src/rostrace/example/scenarios /catkin_ws/src/scenarios

# Build
WORKDIR /catkin_ws
RUN apt-get update && \
    (rosdep init || true) && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace (tool + examples)
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_make"

# Install rostrace as a python package
RUN pip3 install /catkin_ws/src/rostrace

CMD ["bash"]
