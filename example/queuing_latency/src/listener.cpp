#include "ros/ros.h"
#include "std_msgs/String.h"

void topicCallback(const std_msgs::String::ConstPtr &msg) {
  // Let's imagine this callback does some work that takes a bit of time.
  ros::Duration(0.01).sleep(); // Simulate 10ms of work
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("my_topic", 20, topicCallback);
  ros::spin();
  return 0;
}
