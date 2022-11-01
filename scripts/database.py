#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import time
import tf
import math
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from racecar_simulator.msg import CenterPose


class Database():
    def __init__(self, lidar=True):
        # init node
        rospy.init_node('sensor_node')
        # sensor subscriber
        if lidar: rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber("/car_center", CenterPose, self.pose_callback, queue_size=1)
        # Data
        self.lidar_data = None
        self.pose_data = [0,0,0] # x, y, yaw
   
    def lidar_callback(self, data):
        self.lidar_data = data.ranges

    def pose_callback(self, data):
        self.pose_data = data.pose


if __name__ == "__main__":
    try:
        test_db = Database(lidar=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")