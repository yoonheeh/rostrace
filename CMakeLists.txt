cmake_minimum_required(VERSION 3.0.2)
project(rostrace)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rostrace
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the bpf scripts and python module
install(DIRECTORY src/rostrace
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
