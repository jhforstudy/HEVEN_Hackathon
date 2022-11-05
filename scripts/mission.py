#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import threading
from goal import PARKING_SPOT, STOP_LINE, Goal
from parameter_list import Param

from racecar_simulator.msg import CenterPose, HeadPose ,Traffic, Complete
from ackermann_msgs.msg import AckermannDrive

param = Param()
# param.~~~ is a variable in parameter_list.py

# center of parking space
target_x = 0
target_y = 0
target_yaw = 0

# margin for parking space
margin_x = 0
margin_y = 0
margin_yaw = 0


def GetCurrentParkingLot():
    # Target parking lot
    # [0] : first parking lot (x, y, yaw)
    # [1] : second parking lot (x, y, yaw)
    l = [None, None]
    if param.map_3_parking_first_dir == 1:
        l[0] = (param.PARKING_LOT_X_1, param.PARKING_LOT_Y_1, param.PARKING_LOT_YAW_1)
    elif param.map_3_parking_first_dir == 2:
        l[0] = (param.PARKING_LOT_X_2, param.PARKING_LOT_Y_2, param.PARKING_LOT_YAW_2)
    elif param.map_3_parking_first_dir == 3:
        l[0] = (param.PARKING_LOT_X_3, param.PARKING_LOT_Y_3, param.PARKING_LOT_YAW_3)
    elif param.map_3_parking_first_dir == 4:
        l[0] = (param.PARKING_LOT_X_4, param.PARKING_LOT_Y_4, param.PARKING_LOT_YAW_4)

    if param.map_3_parking_second_dir == 5:
        l[1] = (param.PARKING_LOT_X_5, param.PARKING_LOT_Y_5, param.PARKING_LOT_YAW_5)
    elif param.map_3_parking_second_dir == 6:
        l[1] = (param.PARKING_LOT_X_6, param.PARKING_LOT_Y_6, param.PARKING_LOT_YAW_6)
    elif param.map_3_parking_second_dir == 7:
        l[1] = (param.PARKING_LOT_X_7, param.PARKING_LOT_Y_7, param.PARKING_LOT_YAW_7)
    elif param.map_3_parking_second_dir == 8:
        l[1] = (param.PARKING_LOT_X_8, param.PARKING_LOT_Y_8, param.PARKING_LOT_YAW_8)

    return l

class Mission():
    def __init__(self):
        self.position = np.array([0,0])
        self.position_yaw = 0
        self.position_unit_vector = np.array([0,0])

        self.head = np.array([0,0])
        self.head_yaw = 0

        self.speed = 0
        
        self.parked_spots = []
        self.stopped_spots = []
        self.passed_traffic = []

        self.parking_start = False
        self.stop_start = False
        self.traffic_start = False

        self.parking_1_start_time = 0
        self.stop_time = 0

        self.traffic_start_time = 0

        self.parking_index = 0
        self.stop_index = 0

        self.check_flag = 0
        self.success_parking = 0
        self.success_stop = 0

        self.parking_flag = 0

        self.t = None

        park_target_list = GetCurrentParkingLot()

        # ROS settings
        rospy.init_node('mission', anonymous=True)
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        rospy.Subscriber("/drive", AckermannDrive, self.speed_callback, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)
        self.map_number = rospy.get_param('~map_number')
        self.rate = rospy.Rate(param.thread_rate)

        if self.map_number == 2:
            self.goal_list = [
                            # TBD
                            Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180), number=1),
                            Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180), number=2)
                            ]
        
        elif self.map_number == 3:
            self.goal_list = [
                            # TBD
                            Goal(mode=PARKING_SPOT, position=np.array(park_target_list[0][:2]), yaw=math.radians(park_target_list[0][2]), number=1),
                            Goal(mode=PARKING_SPOT, position=np.array(park_target_list[1][:2]), yaw=math.radians(park_target_list[1][2]), number=2),
                            Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180), number=1),
                            Goal(mode=STOP_LINE, position=np.array([100.0, 50.0]), yaw=math.radians(180), number=2)
                            ]
        else:
            rospy.loginfo("Incorrect map_number when making a Goal list!")

        # Thread settings
        # self.t_reached = threading.Thread(target=self.reached)
        # self.t_reached.daemon = True
        # self.t_reached.start()

        self.main()

##########################################################################################################
    def main(self):
        while not rospy.is_shutdown():
            self.reached()
            print(self.check_flag)
            # If mission starts
            if self.check_flag == 1:
                if self.map_number == 2:
                    if self.t.mode == STOP_LINE:
                        self.publish_traffic("stop")
                        if not self.traffic_start:
                            self.stop_index = 0
                            self.traffic_start = True
                        else:
                            self.traffic_mission(self.t)

                elif self.map_number == 3:
                    if self.t.mode == PARKING_SPOT:
                        # First parking
                        if self.t.number == 1:
                            if self.success_parking == 1:
                                # Already Succeed
                                rospy.loginfo("Finished Parking 1. Go ahead.")
                            else:
                                # Before start parking mission
                                if not self.parking_start:
                                    self.parking_index = 0
                                    self.parking_start = True
                                # After start parking
                                else:
                                    self.parking_mission(self.t)
                        # Second parking
                        elif self.t.number == 2:
                            if self.success_parking == 2:
                                # Already Succeed
                                rospy.loginfo("Finished Parking 2. Go ahead.")
                            else:
                                # Before start parking mission
                                if not self.parking_start:
                                    self.parking_index = 0
                                    self.parking_start = True
                                # After start parking
                                else:
                                    self.parking_mission(self.t)
                    elif self.t.mode == STOP_LINE:
                        self.publish_traffic("stop")
                        # TBD
                        if self.stop_start == False:
                            self.stop_index = 0
                            self.stop_start = True
                        else:
                            self.stop_mission(self.t)
                    else:
                        rospy.loginfo("WRONG GOAL FORMAT !!!")

            self.rate.sleep()
##########################################################################################################

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

    # Check if parking mission is end
    def parking_mission(self, goal=Goal):
        # goal.position[0] = x coordinate / goal.position[1] = y coordinate / goal.yaw = yaw angle (degree)
        parking_spot = goal.position
        
        if self.parking_index == 0:
            # Check if a car is stopped
            if goal.mode == PARKING_SPOT and self.speed == 0:
                self.parking_1_start_time = time.time()
                self.parking_index += 1

        elif self.parking_index == 1:
            # After a car stopped
            if time.time() - self.parking_1_start_time >= 3:
                # Parking success
                rospy.loginfo("Parking mission success!")
                self.parking_flag = 1
                self.parking_index += 1

            elif time.time() - self.parking_1_start_time < 3 and self.speed != 0:
                # Parking fail
                rospy.loginfo("Parking mission failed...")
                self.parking_flag = 0
                parking_spot = None
                self.parking_index += 1
        
        elif self.parking_index == 2:
            # parking misison failed, break the function
            if parking_spot is None:
                self.parking_start = False
            # parking mission success
            else:
                self.parking_index += 1
    
        elif self.parking_index == 3:
            self.success_parking += self.parking_flag
            self.parked_spots.append(parking_spot)
        
            # publish that parking mission is succeed
            complete_msg = Complete()
            complete_msg.complete = True
            self.complete.publish(complete_msg)

            # Reset the trigger
            self.parking_start = False
            self.parking_flag = 0

    #### left time and stop publish
    def stop_mission(self, goal=Goal):
        stop_spot = goal.position
        stop_flag = 0
        traffic = Traffic()
        
        if self.stop_index == 0:
            # check if a car is stopped
            if goal.mode == STOP_LINE and self.speed == 0:
                self.stop_time = time.time()
                self.stop_index += 1
        
        elif self.stop_index == 1:
            # After a car stopped
            if time.time() - self.stop_time >= 3:
                # Stop success
                rospy.loginfo("Stop mission success!")
                stop_flag = 1
                self.stop_index += 1

            elif time.time() - self.stop_time < 3 and self.speed != 0:
                # Stop fail
                rospy.loginfo("Stop mission failed...")
                stop_spot = None
                self.stop_index += 1

        elif self.stop_index == 2:
            # stop mission failed, break the function
            if stop_spot is None:
                self.stop_start = False
            # stop misison success
            else:
                self.stop_index += 1
        
        elif self.stop_index == 3:
            # Stop mission finally end
            self.success_stop += stop_flag
            self.stopped_spots.append(stop_spot)
        
            # publish that stop mission is succeed
            complete_msg = Complete()
            complete_msg.complete = True
            self.complete.publish(complete_msg)
            
            # Reset the trigger
            self.stop_start = False

        """
        if self.t is not None:
            if self.t.mode == STOP_LINE and self.speed == 0:
                stop_succeed = 1
                stop_spot = self.t.position
                while time.time() - self.start_time <= 3:
                    if self.t.mode != STOP_LINE:
                        stop_succeed = 0
                        stop_spot = None
                        break
                    else:
                        pass
                if stop_spot is not None:
                    self.complete.publish(True)
        
        if stop_spot not in self.stopped_spots:
            self.success_flag += stop_succeed
            self.stop_spots.append(stop_spot)
        """

    # Check if traffic mission is end
    def traffic_mission(self, goal=Goal):
        traffic_spot = goal.position
        traffic_flag = 0
        if self.stop_index == 0:
            #check if a car is stopped
            if goal.mode == STOP_LINE and self.speed == 0:
                self.stop_time = time.time()
                self.stop_index += 1
        
        elif self.stop_index == 1:
            # After a car stopped
            if time.time() - self.stop_time >= 3:
                # Stop success
                rospy.loginfo("Stop mission success!")
                stop_flag = 1
                self.stop_index += 1
            
            elif time.time() - self.stop_time < 3 and self.speed != 0:
                # Stop fail
                rospy.loginfo("Stop mission failed...")
                stop_spot = None
                self.stop_index += 1

        elif self.stop_index == 2:
            # stop mission failed, break the function
            if stop_spot is None:
                self.stop_start = False
            # stop misison success
            else:
                self.stop_index += 1
        
        elif self.stop_index == 3:
            while self.check(goal):
                self.publish_traffic(param.map_2_traffic_dir)
        else:
            pass
        """
        elif self.stop_index == 2:
            # stop mission failed, break the function
            if stop_spot is None:
                self.stop_start = False
            # stop misison success
            else:
                self.stop_index += 1
        
        elif self.stop_index == 3:
            if stop_spot not in self.stopped_spots:
                self.success_flag += stop_flag
                self.stopped_spots.append(stop_spot)
            
                # publish that stop mission is succeed
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)
        

        if self.t is not None:
            if self.traffic_index == 0 and self.t.mode == STOP_LINE and self.speed == 0:
                stop_succeed = 1
                stop_spot = self.t.position
                while time.time() - self.start_time <= 3:
                    if self.t.mode != STOP_LINE:
                        stop_succeed = 0
                        stop_spot = None
                        break
                    else:
                        pass
                if stop_spot is not None:
                    self.traffic_index += 1
                
            while self.traffic_index == 1 and self.t.mode == STOP_LINE:
                self.traffic.publish(param.map_2_traffic_dir)
                
        if self.t is not None:
            if self.t.mode == STOP_LINE and self.speed == 0:
                traffic_spot = self.t.position
                traffic_succeed = 1
                while time.time() - self.start_time <= 3:
                    if self.t.mode != STOP_LINE:
                        traffic_succeed = 0
                        break
                    else:
                        pass
                if traffic_succeed == 1:
                    complete_msgs = Complete()
                    complete_msgs.complete = True
                    self.complete.publish(complete_msgs)
        if traffic_spot not in self.passed_traffic:
            self.success_flag += traffic_succeed
            self.traffic_spots.append(traffic_spot)
        """

    def check(self, goal=Goal):
        check_flag = 0

        if goal is not None:
            if goal.mode == PARKING_SPOT:
                # Check if a car is in the parking lot
                position_diff = goal.position - self.position
                yaw_diff = goal.yaw - math.radians(self.position_yaw)
                # What t f?????
                # rot_ref = goal.yaw * np.array(-position_diff)
                if abs(position_diff[0]) <= goal.tolarance[0] and abs(position_diff[1]) <= goal.tolarance[1] and abs(yaw_diff) <= goal.tolarance[2]:
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
        else:
            pass

        self.check_flag = check_flag

        return check_flag

    # Check if a car is in Mission area
    def reached(self):
        reached_target = None
        for target in self.goal_list:
            if self.check(target) == 1:
                reached_target = target
                break
        
        self.t = reached_target
            
    def show_result(self, parking_spots, stop_spots):
        """TBD - Show result of mission"""
        print("Parking spots : ", parking_spots)
        print("Stop spots : ", stop_spots)
        
    def publish_traffic(self, what):
        traffic = Traffic()
        traffic.traffic = what
        self.traffic.publish(traffic)

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