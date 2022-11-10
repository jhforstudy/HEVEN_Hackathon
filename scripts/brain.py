#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time

from database import Database
from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param

param = Param()

class Brain():
    def __init__(self, db=Database, map_number=int):
        self.db = db
        self.map_number = map_number
        self.parking_x_1 = param.parking_x_1
        self.parking_y_1 = param.parking_y_1
        self.parking_yaw_1 = param.parking_yaw_1
        self.parking_x_2 = param.parking_x_2
        self.parking_y_2 = param.parking_y_2
        self.parking_yaw_2 = param.parking_yaw_2
    
    def main(self):
        # You can choose map number using this (1, 2, 3)
        # Please erase this when you submit the code!!!
        self.map_number = 1
        # ===============================================
        '''
        # 1. How to call sensor data?
        lidar_data = self.db.lidar_data

        # 2. How to call pose data?
        pose_data = self.db.pose_data [x, y, yaw(degree)]

        # 3. How to get info of traffic light?
        traffic_light = self.db.traffic_light
        remaining_time = self.db.traffic_remaining_time

        # 4. How to get the position of parking lot?
        park_x_1 = self.parking_x_1
        park_y_1 = self.parking_y_1
        park_yaw_1 = self.parking_yaw_1

        # finally derive the angle & speed of a car
        return angle, speed
        '''
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data
        traffic_light = self.db.traffic_light
        remaining_time = self.db.traffic_remaining_time

        # Print for debugging
        print(traffic_light, remaining_time)

        # Determine the angle & speed
        angle = 0
        speed = 0
        # Return angle & speed
        return angle, speed


if __name__ == "__main__":
    db = Database(lidar=True)
    test_brain = Brain(db)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('drive', AckermannDrive, queue_size=1)
    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        motor_msg = AckermannDrive()
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        motor_msg.steering_angle_velocity = param.car_angular_velocity
        motor_msg.acceleration = param.car_acceleration
        motor_msg.jerk = param.car_jerk
        rate.sleep()