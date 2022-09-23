#!/usr/bin/env python

import rospy
import time

from database import Database
from brain import Brain

from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

CAR_ANGLE_VELOCITY = 1
CAR_ACCELERATION = 1
CAR_JERK = 0

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

    # whole_msg = AckermannDriveStamped()
    # # Header
    # whole_msg.header.stamp = rospy.Time.now()
    # whole_msg.header.frame_id = '/race_car'
    # whole_msg.drive = control_msg
    # AckermannDrive
    control_msg = AckermannDrive()
    control_msg.steering_angle = curr_angle
    control_msg.steering_angle_velocity = CAR_ANGLE_VELOCITY
    control_msg.speed = curr_speed
    control_msg.acceleration = CAR_ACCELERATION
    control_msg.jerk = CAR_JERK

    return control_msg

def main():
    # Initialize database
    db = Database(lidar=True)
    brain = Brain(db)
    # Initialize ROS rate & motor publisher
    rate = rospy.Rate(100)
    control_pub = rospy.Publisher('drive', AckermannDrive, queue_size=1)
    rospy.loginfo("---Initializing mission manager---\n\n\n")
    time.sleep(1)

    rospy.loginfo("---Done. Start autonomous driving---\n\n\n")
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