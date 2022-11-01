#!/usr/bin/env python

from ast import arg
from tkinter import X
import rospy
import time
import math
from goal import PARKING_SPOT, STOP_LINE, Goal
import numpy as np
import threading

from racecar_simulator.msg import CenterPose, HeadPose ,Traffic

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
        self.position_x = 0
        self.position_y = 0
        self.position_yaw = 0

        self.head_x = 0
        self.head_y = 0
        self.head_yaw = 0
        
        self.success_flag = 0
        self.parked_spots = []

        self.goal_list = [
                        Goal(mode=PARKING_SPOT, x=100.0, y=50.0, yaw=math.radians(180)),
                        Goal(mode=PARKING_SPOT, x=150.0, y=50.0, yaw=math.radians(45)),
                        Goal(mode=STOP_LINE, x=0.0, y=0.0, yaw=math.radians(180)),
                        Goal(mode=STOP_LINE, x=0.0, y=0.0, yaw=math.radians(180))
                        ]

        # ROS settings
        rospy.init_node('mission', anonymous=True)
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)

        self.t = threading.Thread(target=self.reached)
        self.t.daemon = True
        self.t.start()

        pass
    
    def parking(self):
        """ have to modify"""
        if abs(self.position_x - Goal.target[0][0]) < margin_x and abs(self.position_y - Goal.target[0][1]) < margin_y and abs(self.position_yaw - Goal.target[0][2]) < margin_yaw:
            start_time = time.time()
            while time.time() - start_time == 3:
                self.success_flag += 1
        
        elif abs(self.position_x - Goal.target[1][0]) < margin_x and abs(self.position_y - Goal.target[1][1]) < margin_y and abs(self.position_yaw - Goal.target[1][2]) < margin_yaw:
            start_time = time.time()
            while time.time() - start_time == 3:
                self.success_flag += 1
        
        elif abs(self.position_x - Goal.target[2][0]) < margin_x and abs(self.position_y - Goal.target[2][1]) < margin_y and abs(self.position_yaw - Goal.target[2][2]) < margin_yaw:
            start_time = time.time()
            while time.time() - start_time == 3:
                self.success_flag += 1

    def traffic(self):
        if self.head_x
        2.1
        # self.traffic.publish(self.traffic_data)
        pass

    def main(self):
        self.parking()
        self.traffic()
        self.check()
        pass

    def position_callback(self, data):
        self.position_x = data.pose[0]
        self.position_y = data.pose[1]
        self.position_yaw = data.pose[2]

    def head_callback(self, data):
        self.head_x = data.pose[0]
        self.head_y = data.pose[1]
        self.head_yaw = data.pose[2]



    def check(self, goal):
        check_flag = 0
        x_diff = goal.x - self.position_x
        y_diff = goal.y - self.position_y
        yaw_diff = goal.yaw - self.position_yaw

        if goal.mode == PARKING_SPOT:
            x_diff = goal.x - self.position_x
            y_diff = goal.y - self.position_y
            yaw_diff = goal.yaw - self.position_yaw

            rot_ref = goal.rotation * np.array([x_diff, y_diff])
            _x_diff = rot_ref[0]
            _y_diff = rot_ref[1]

            if abs(_x_diff) <= goal.tolarance[0] and abs(_y_diff) <= goal.tolarance[1] and abs(yaw_diff) <= goal.tolarance[2]:
                check_flag = 1
            
        elif goal.mode == STOP_LINE:
            x_diff = goal.x - self.head_x
            y_diff = goal.y - self.head_y
            yaw_diff = goal.yaw - self.head_yaw

            dist = math.sqrt(math.pow(x_diff,2) + math.pow(y_diff,2))
            angle = goal.yaw - math.atan2(-x_diff, -y_diff)
            if angle > math.pi:
                angle = angle - 2*math.pi
            elif angle < -math.pi:
                angle = angle + 2*math.pi
            
            if dist <= goal.tolarance[0] and abs(yaw_diff) <= math.pi/2 and abs(angle) < math.pi/2:
                check_flag = 1
                     
        else:
            print("wrong goal format")

        return check_flag

    def reached(self):
        reached_target = None
        for target in self.goal_list:
            if self.check(target) == 1:
                reached_target = target
                break
        return reached_target
        # 이 함수가 반복문 안에서 돌면 됨.

    def parking_mission(self):
        if self.t is not None:
            if self.t.mode == PARKING_SPOT:
                parking_succeed = 1
                parking_spot = (self.t.x, self.t.y)
                start_time = time.time()
                while time.time() - start_time == 3:
                    if self.t.mode != PARKING_SPOT:
                        parking_succeed = 0
                        parking_spot = 0
                        break
                    else:
                        pass
        if parking_spot not in self.parked_spots:
            self.success_flag += parking_succeed
            self.parked_spots.append(parking_spot)

        return

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