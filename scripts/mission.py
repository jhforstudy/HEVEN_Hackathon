#!/usr/bin/env python

# -*- coding: utf-8 -*-

from tkinter import X
import rospy
import time
import math
from goal import PARKING_SPOT, STOP_LINE, Goal
import numpy as np
import threading

from racecar_simulator.msg import CenterPose, HeadPose ,Traffic
from ackermann_msgs.msg import AckermannDrive

# parking_space_x = 0.65 (maybe 0.65/sqrt(2))
# parking_space_y = 0.45 (maybe 0.45/sqrt(2))
# parking_space_yaw = 90 (maybe 45)

# center of parking space
target_x = 0
target_y = 0
target_yaw = 0

# margin for parking space
margin_x = 0
margin_y = 0
margin_yaw = 0


class Mission():
    def __init__(self):
        self.position = np.array([0,0])
        self.position_yaw = 0
        self.position_unit_vector = np.array([0,0])

        self.head = np.array([0,0])
        self.head_yaw = 0

        self.speed = 0
        
        self.success_flag = 0
        self.parked_spots = []
        self.stop_spots = []

        self.goal_list = [
                        # TBD
                        Goal(mode=PARKING_SPOT, position=np.array([100.0, 50.0]), yaw=math.radians(180)),
                        Goal(mode=PARKING_SPOT, position=np.array([100.0, 50.0]), yaw=math.radians(45)),
                        Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180)),
                        Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180))
                        ]

        # ROS settings
        rospy.init_node('mission', anonymous=True)
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        rospy.Subscriber("/drive", AckermannDrive, self.speed_callback, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)

        # Thread settings
        self.t = threading.Thread(target=self.reached)
        self.t.daemon = True
        self.t.start()

    def main(self):
        self.parking_mission()
        # self.traffic_misison()
        self.check()
        pass

    def position_callback(self, data):
        self.position = np.array([data.pose[0], data.pose[1]])
        self.position_yaw = data.pose[2] # yaw angle
        self.position_unit_vector = np.array([math.cos(data.pose[2]), math.sin(data.pose[2])]) # yaw unit vector
        
    def head_callback(self, data):
        self.head = np.array([data.pose[0], data.pose[1]])
        self.head_yaw = data.pose[2] # yaw angle
        self.head_unit_vector = np.array([math.cos(data.pose[2]), math.sin(data.pose[2])]) # yaw unit vector

    def speed_callback(self, data):
        self.speed = data.speed

    def parking_mission(self):
        # self.t.x = x coordinate / self.t.y = y coordinate / self.t.yaw = yaw angle (degree)
        if self.t is not None:
            if self.t.mode == PARKING_SPOT and self.speed == 0:
                parking_succeed = 1
                parking_spot = self.t.position
                start_time = time.time()
                while time.time() - start_time <= 3:
                    if self.t.mode != PARKING_SPOT:
                        parking_succeed = 0
                        parking_spot = None
                        break
                    else:
                        pass
        if parking_spot not in self.parked_spots:
            self.success_flag += parking_succeed
            self.parked_spots.append(parking_spot)

    def stop_mission(self):
        if self.t is not None:
            if self.t.mode == STOP_LINE and self.speed == 0:
                stop_succeed = 1
                stop_spot = self.t.position
                start_time = time.time()
                while time.time() - start_time <= 3:
                    if self.t.mode != STOP_LINE:
                        stop_succeed = 0
                        stop_spot = None
                        break
                    else:
                        pass
        if stop_spot not in self.stop_spots:
            self.success_flag += stop_succeed
            self.stop_spots.append(stop_spot)

    def check(self, goal=Goal):
        check_flag = 0

        if goal.mode == PARKING_SPOT:
            # Check if parking is end
            position_diff = goal.position - self.position
            yaw_diff = goal.yaw - self.position_yaw
            
            rot_ref = goal.yaw * np.array(-position_diff)

            if abs(rot_ref[0]) <= goal.tolarance[0] and abs(rot_ref[1]) <= goal.tolarance[1] and abs(yaw_diff) <= goal.tolarance[2]:
                check_flag = 1
            
        elif goal.mode == STOP_LINE:
            # Check if a car passes stop line
            position_diff = goal.position - self.head
            angle = goal.yaw - math.atan2(-position_diff[0], -position_diff[1])
            unit_vector_diff = np.array([math.cos(angle), math.sin(angle)])
            dist = np.linalg.norm(position_diff)

            if np.dot(self.position_unit_vector, goal.unit_vector)<=0\
                and np.dot(unit_vector_diff, goal.unit_vector)<=0\
                and dist <= goal.tolarance[0]:
                check_flag = 1
                     
        else:
            rospy.loginfo("WRONG GOAL FORMAT !!!")

        return check_flag

    # Check if a car is in Mission area
    def reached(self):
        reached_target = None
        for target in self.goal_list:
            if self.check(target) == 1:
                reached_target = target
                break
        return reached_target

if __name__ == "__main__":
    try:
        test_mission = Mission()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                test_mission.main()
                rate.sleep()
            except :
                continue
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")