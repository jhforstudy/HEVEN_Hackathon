#!/usr/bin/env python

import rospy
import time
import tf
import math

from database import Database
from brain import Brain
from tf.transformations import euler_from_quaternion

from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param

param = Param()

def drive(curr_angle, curr_speed):
    '''
    ackermann_msgs/AckermannDriveStamped.msg

    Header          header
    AckermannDrive  drive

    ackermann_msgs/AckermannDrive.msg

    float32 steering_angle
    float32 steering_angle_velocity
    float32 speed
    float32 acceleration
    float32 jerk
    '''
    
    control_msg = AckermannDrive()
    control_msg.steering_angle = curr_angle*math.pi/180
    control_msg.steering_angle_velocity = param.car_angular_velocity
    control_msg.speed = curr_speed
    control_msg.acceleration = param.car_acceleration
    control_msg.jerk = param.car_jerk

    return control_msg

def main():
    # Initialize database
    db = Database(lidar=True)
    map_number = rospy.get_param('~map_number', 1)
    brain = Brain(db, map_number=map_number)
    # Initialize ROS rate & motor publisher
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('drive', AckermannDrive, queue_size=1)
    rospy.loginfo("Start autonomous driving---\n\n\n")
    time.sleep(1)

    while not rospy.is_shutdown():
        # return the speed and angle
        angle, speed = brain.main()
        # Publish control data
        control_pub.publish(drive(angle, speed))
        # wait
        rate.sleep()

if __name__ == "__main__":
    main()