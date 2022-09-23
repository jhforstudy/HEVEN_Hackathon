#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import time

from sensor_msgs.msg import LaserScan


class Database():
    def __init__(self, lidar=True):
        # init node
        rospy.init_node('sensor_node')
        rospy.loginfo("---Initializing sensor node---\n\n\n")
        time.sleep(1)
        # sensor subscriber
        if lidar: rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        # Data
        self.lidar_data = None
        rospy.loginfo("---now subscribing sensor data---\n\n\n")
        time.sleep(1)
   
    def lidar_callback(self, data):
        self.lidar_data = data.ranges


if __name__ == "__main__":
    try:
        test_db = Database(lidar=True)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")