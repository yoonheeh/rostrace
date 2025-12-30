#include <chrono>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>

// Scenario: High Processing Latency
// Cause: The callback takes too long to execute.

void chatterCallback(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Processing message: [%s]", msg->data.c_str());

  // Simulate heavy computation or blocking I/O
  // This will show up as "Processing Latency" in rostrace
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  ROS_INFO("Finished processing.");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "processing_delay_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("slow_topic", 1000, chatterCallback);

  ros::spin();

  return 0;
}
