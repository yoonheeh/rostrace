#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <chrono>

// Scenario: High Queuing Latency
// Cause: The main thread is busy handling a service call, so the topic callback
//        cannot start even though the message has arrived and is in the queue.

bool heavyService(std_srvs::Empty::Request  &req,
                  std_srvs::Empty::Response &res)
{
  ROS_INFO("Service called! Blocking thread for 500ms...");
  // Simulate a long-running service task (e.g., complex calculation, hardware sync)
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ROS_INFO("Service finished.");
  return true;
}

void topicCallback(const std_msgs::String::ConstPtr& msg)
{
  // This callback is fast, but it will be delayed if the service is running.
  ROS_INFO("Topic received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_blocker_node");
  ros::NodeHandle n;

  // Single-threaded spinner (default) means we can only do one thing at a time.
  
  ros::ServiceServer service = n.advertiseService("heavy_service", heavyService);
  ros::Subscriber sub = n.subscribe("blocked_topic", 1000, topicCallback);

  ROS_INFO("Ready to demonstrate Queuing Latency. Call /heavy_service to block the thread.");
  
  ros::spin();

  return 0;
}
