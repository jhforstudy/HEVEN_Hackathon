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
    def __init__(self, db):
        self.db = db
    
    def main(self):
        '''
        # How to call sensor data
        lidar_data = self.db.lidar_data

        # How to call pose data
        pose_data = self.db.pose_data

        # Add other functions or algorithm
        self.example_function()

        # finally derive the angle & speed of a car
        return angle, speed
        '''
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data
        
        angle = 0
        speed = 0

        return angle, speed


if __name__ == "__main__":
    db = Database(lidar=True)
    test_brain = Brain(db)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('drive', AckermannDrive, queue_size=1)
    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        # print(car_angle, car_speed)

        motor_msg = AckermannDrive()
        # print("car_speed",car_speed)
        # print("car_angle",car_angle)
        # print("car_speed: car_speed, angle: {car_angle}")
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        motor_msg.steering_angle_velocity = param.car_angular_velocity
        motor_msg.acceleration = param.car_acceleration
        motor_msg.jerk = param.car_jerk

        rate.sleep()