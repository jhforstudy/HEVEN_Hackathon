#!/usr/bin/env python

from math import pi
import rospy
import numpy as np
import time
import cv2

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose

WHEELBASE = 0.425
REAR_LIDAR = 0.325
WIDTH = 0.145

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
        rospy.Subscriber('scan', LaserScan, self.callback)

    def callback(self, data):
        self.lidar_data = np.array(data.ranges)

class Count():
    def __init__(self):
        self.initial_clock = rospy.get_time()
        self.collision_count = 0
        pass


# If collide, Move a car to Initialpose
def collision_detection(lidar_data):
    min_distance = np.min(np.array(lidar_data) - np.array(check_array))
    if min_distance <= 0.01:
        return True
    else:
        return False

if __name__ == "__main__":
    rospy.init_node("Check_collision")
    pose_pub = rospy.Publisher('pose', Pose, queue_size=1)
    check_col = CheckCollide()
    count = Count()
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
            count.collision_count += 1

        elapsed_time = round((rospy.get_time() - count.initial_clock), 3)
        text_1 = "Time : " + str(elapsed_time) + " sec"
        text_2 = "Collision : " + str(count.collision_count)

        img = np.full(shape=(100,400,3),fill_value=0,dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX

        cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
        cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
        cv2.imshow("image",img)
        cv2.moveWindow('image', 0, 0)
        cv2.waitKey(1)

        rate.sleep()

    cv2.destroyAllWindows()