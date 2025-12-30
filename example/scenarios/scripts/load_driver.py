#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
import threading
import time

def call_service_loop():
    rospy.wait_for_service('heavy_service')
    heavy_service = rospy.ServiceProxy('heavy_service', Empty)
    rate = rospy.Rate(1) # Call service every 1 second
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Calling /heavy_service...")
            heavy_service()
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed: {e}")
        rate.sleep()

def main():
    rospy.init_node('load_driver')

    # Publishes to the slow processing node
    pub_process = rospy.Publisher('slow_topic', String, queue_size=10)
    
    # Publishes to the service blocker node
    pub_block = rospy.Publisher('blocked_topic', String, queue_size=10)

    # Start a background thread to call the service continuously
    t = threading.Thread(target=call_service_loop)
    t.daemon = True
    t.start()

    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        msg = "data"
        
        # 1. Trigger Processing Latency
        pub_process.publish(msg)
        
        # 2. Trigger Queuing Latency (will be blocked by service calls)
        pub_block.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
