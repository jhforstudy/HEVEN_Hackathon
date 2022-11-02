#!/usr/bin/env python

from math import pi
import rospy
import numpy as np
import time
import cv2

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from racecar_simulator.msg import CenterPose
from parameter_list import Param

param = Param()

check_array = np.zeros(shape=360)
for i in range(0, 26):
    check_array[i] = param.REAR_LIDAR / np.cos(i*pi/180)

for i in range(26, 123):
    check_array[i] = param.WIDTH / np.sin(i*pi/180)

for i in range(123, 181):
    check_array[i] = - (param.WHEELBASE - param.REAR_LIDAR) / np.cos(i*pi/180)

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
        rospy.Subscriber('car_center', CenterPose, self.pose_callback)

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
def end_detection(pose_data, mission_number):
    if mission_number == 1:
        length = np.array(pose_data) - np.array([param.END_POINT_X_1, param.END_POINT_Y_1])
        norm = np.linalg.norm(length)
    elif mission_number == 2:
        length = np.array(pose_data) - np.array([param.END_POINT_X_2, param.END_POINT_Y_2])
        norm = np.linalg.norm(length)
    elif mission_number == 3:
        length = np.array(pose_data) - np.array([param.END_POINT_X_3, param.END_POINT_Y_3])
        norm = np.linalg.norm(length)
    else:
        norm = 0
        rospy.loginfo("Mission number is incorrect.")

    if norm < param.SIZE_OF_TROPHY:
        return True
    else:
        return False

if __name__ == "__main__":
    rospy.init_node("Check_collision")
    pose_pub = rospy.Publisher('pose', Pose, queue_size=1)
    check_col = CheckCollide()
    check_end = CheckEnd()
    rate = rospy.Rate(param.thread_rate)
    # Get mission number
    mission_number = rospy.get_param('~mission_number')
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

        if end_detection(check_end.pose_data[:2], mission_number):
            rospy.loginfo("Time : %.3f", elapsed_time)
            rospy.loginfo("Collision : %d", check_col.collision_count)
            rospy.signal_shutdown("reason")

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