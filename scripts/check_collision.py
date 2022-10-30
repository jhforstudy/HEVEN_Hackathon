#!/usr/bin/env python

from math import pi
import rospy
import numpy as np
import time
import cv2

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from racecar_simulator.msg import CarPose

WHEELBASE = 0.425
REAR_LIDAR = 0.325
WIDTH = 0.145
SIZE_OF_TROPHY = 0.5

# Endpoint of Map 1
END_POINT_X = 13.34
END_POINT_Y = -9.31

check_array = np.zeros(shape=360)
for i in range(0, 26):
    check_array[i] = REAR_LIDAR / np.cos(i*pi/180)

for i in range(26, 123):
    check_array[i] = WIDTH / np.sin(i*pi/180)

for i in range(123, 181):
    check_array[i] = - (WHEELBASE - REAR_LIDAR) / np.cos(i*pi/180)

for i in range(181, 360):
    check_array[i] = check_array[360-i]

class CheckCollide():
    def __init__(self):
        self.lidar_data = None
        self.initial_clock = rospy.get_time()
        self.collision_count = 0
        rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        self.lidar_data = np.array(data.ranges)

class CheckEnd():
    def __init__(self):
        self.if_end = False
        self.pose_data = None
        rospy.Subscriber('car_pose', CarPose, self.pose_callback)

    def pose_callback(self, data):
        self.pose_data = np.array(data.pose)

# If collide, Move a car to Initialpose
def collision_detection(lidar_data):
    min_distance = np.min(np.array(lidar_data) - np.array(check_array))
    if min_distance <= 0.01:
        return True
    else:
        return False

# If end, stop the mission and show the results
def end_detection(pose_data):
    length = np.array(pose_data) - np.array([END_POINT_X, END_POINT_Y])
    norm = np.linalg.norm(length)

    if norm < SIZE_OF_TROPHY:
        return True
    else:
        return False

if __name__ == "__main__":
    rospy.init_node("Check_collision")
    pose_pub = rospy.Publisher('pose', Pose, queue_size=1)
    check_col = CheckCollide()
    check_end = CheckEnd()
    rate = rospy.Rate(10)
    time.sleep(1)
    while not rospy.is_shutdown():
        
        if collision_detection(check_col.lidar_data):
            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_pub.publish(pose)
            check_col.collision_count += 1

        if end_detection(check_end.pose_data[:2]):
            print("Time : ", elapsed_time)
            print("Collision : ", check_col.collision_count)
            break

        elapsed_time = round((rospy.get_time() - check_col.initial_clock), 3)
        text_1 = "Time : " + str(elapsed_time) + " sec"
        text_2 = "Collision : " + str(check_col.collision_count)

        img = np.full(shape=(100,400,3),fill_value=0,dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
        cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
        cv2.imshow("image",img)
        cv2.moveWindow('image', 0, 0)
        cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()