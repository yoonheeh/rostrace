# Dockerfile for the rostrace tool (CLI only)
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

# Copy only the package source code
# We avoid copying 'demo' or 'example' folders here
COPY package.xml CMakeLists.txt setup.py /catkin_ws/src/rostrace/
COPY src /catkin_ws/src/rostrace/src
COPY scripts /catkin_ws/src/rostrace/scripts

# Build
WORKDIR /catkin_ws
RUN apt-get update && \
    (rosdep init || true) && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the catkin workspace (rostrace tool only)
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_make"

# Install rostrace as a python package so it is available globally
RUN pip3 install /catkin_ws/src/rostrace

# No ENTRYPOINT - let the user define the command
CMD ["bash"]
