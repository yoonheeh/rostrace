cmake_minimum_required(VERSION 3.0.2)
project(scenarios)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Node 1: Processing Delay Demo
add_executable(processing_delay_node src/processing_delay_node.cpp)
target_link_libraries(processing_delay_node ${catkin_LIBRARIES})

# Node 2: Service Blocker Demo (Queuing Latency)
add_executable(service_blocker_node src/service_blocker_node.cpp)
target_link_libraries(service_blocker_node ${catkin_LIBRARIES})

# Install targets
install(TARGETS processing_delay_node service_blocker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/load_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
