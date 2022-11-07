#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import threading
import tkinter as tk
import cv2
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

def GetStopLine():
    l = [None, None]
    l[0] = (param.MAP_3_STOP_LINE_X_1, param.MAP_3_STOP_LINE_Y_1, param.MAP_3_STOP_LINE_YAW_1)
    l[1] = (param.MAP_3_STOP_LINE_X_2, param.MAP_3_STOP_LINE_Y_2, param.MAP_3_STOP_LINE_YAW_2)

    return l

def dis(pos_1=np.array, pos_2=np.array):
    position_diff = pos_1 - pos_2
    dist = np.linalg.norm(position_diff)
    return dist

class Mission():
    def __init__(self):
        # Information of a car
        self.position = np.array([0,0])
        self.position_yaw = 0
        self.position_unit_vector = np.array([0,0])
        self.head = np.array([0,0])
        self.head_yaw = 0
        self.speed = 0
        
        # Save mission spots
        self.parked_spots = []
        self.stopped_spots = []
        self.passed_traffic = []

        # Check if mission starts
        self.parking_start = False
        self.stop_start = False
        self.traffic_start = False

        # For parking
        self.parking_success = False
        self.parking_start_time = 0
        self.parking_index = 0
        self.parking_flag = 0

        # For stop mission
        self.stop_success = False
        self.stop_time = 0
        self.stop_index = 0
        self.stop_flag = 0

        # For traffic mission
        self.traffic_success = False
        self.traffic_start_time = 0
        self.traffic_index = 0
        self.traffic_flag = 0

        # Check if a car is in the mission area
        # If it is, become 1
        self.check_flag = 0

        # Number of missions succeed or passed
        self.num_passed_traffic = 0
        self.num_success_traffic = 0
        self.num_success_parking = [0,0]
        self.num_success_stop = [0,0]

        # If spawn point is updated
        self.parking_updated = False
        self.stop_updated = False

        # Current goal
        self.t = None

        # Get the 2 points of parking lot from parameter_list.py
        park_target_list = GetCurrentParkingLot()
        stop_line_list = GetStopLine()

        # ROS settings
        rospy.init_node('mission', anonymous=True)
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        rospy.Subscriber("/drive", AckermannDrive, self.speed_callback, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)
        self.complete = rospy.Publisher("/complete", Complete, queue_size=1)
        self.map_number = rospy.get_param('~map_number')

        if self.map_number == 1:
            self.goal_list = []

        elif self.map_number == 2:
            self.goal_list = [
                            Goal(mode=STOP_LINE, position=np.array((param.MAP_2_STOP_LINE_X, param.MAP_2_STOP_LINE_Y)), yaw=math.radians(90), number=1)
                            ]
        
        elif self.map_number == 3:
            self.goal_list = [
                            # TBD
                            Goal(mode=PARKING_SPOT, position=np.array(park_target_list[0][:2]), yaw=math.radians(park_target_list[0][2]), number=1),
                            Goal(mode=PARKING_SPOT, position=np.array(park_target_list[1][:2]), yaw=math.radians(park_target_list[1][2]), number=2),
                            Goal(mode=STOP_LINE, position=np.array(stop_line_list[0][:2]), yaw=math.radians(stop_line_list[0][2]), number=1),
                            Goal(mode=STOP_LINE, position=np.array(stop_line_list[1][:2]), yaw=math.radians(stop_line_list[1][2]), number=2)
                            ]
        else:
            rospy.loginfo("Incorrect map_number when making a Goal list!")

        # Thread for Init button
        button_mission_thread = threading.Thread(target=self.init_mission_button)
        button_mission_thread.daemon = True
        button_mission_thread.start()

##########################################################################################################
    def main(self):
        # Check if a car is any mission area
        # if true, return self.check_flag = 1
        self.reached()
        # If any mission starts
        if self.check_flag == 1:
            # map 2
            if self.map_number == 2:
                if self.t.mode == STOP_LINE:
                    if self.num_passed_traffic == 1:
                        # Already Succeed
                        rospy.loginfo("Finished Traffic mission. Go ahead.")
                    else:
                        # Before start parking mission
                        if not self.traffic_start:
                            self.traffic_index = 0
                            self.traffic_start = True
                        # After start parking
                        else:
                            self.traffic_mission(self.t)

            # map 3
            elif self.map_number == 3:
                if self.t.mode == PARKING_SPOT:
                    # First parking
                    if self.t.number == 1:
                        if self.num_success_parking[0] == 1:
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
                        if self.num_success_parking[1] == 1:
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
                    # First stop
                    if self.t.number == 1:
                        if self.num_success_stop[0] == 1:
                            # Already Succeed
                            rospy.loginfo("Finished Stop 1. Go ahead.")
                            traffic_msg = Traffic()
                            traffic_msg.traffic = "GO"
                            self.traffic.publish(traffic_msg)
                        else:
                            # Before start parking mission
                            if not self.stop_start:
                                self.stop_index = 0
                                self.stop_start = True
                            # After start parking
                            else:
                                self.stop_mission(self.t)
                    # Second stop
                    elif self.t.number == 2:
                        if self.num_success_stop[1] == 1:
                            # Already Succeed
                            rospy.loginfo("Finished Stop 2. Go ahead.")
                            traffic_msg = Traffic()
                            traffic_msg.traffic = "GO"
                            self.traffic.publish(traffic_msg)
                        else:
                            # Before start parking mission
                            if not self.stop_start:
                                self.stop_index = 0
                                self.stop_start = True
                            # After start parking
                            else:
                                self.stop_mission(self.t)
                else:
                    rospy.loginfo("WRONG GOAL FORMAT !!!")

        self.visualize_mission()
##########################################################################################################

    def position_callback(self, data):
        self.position = np.array([data.pose[0], data.pose[1]])
        self.position_yaw = (data.pose[2]) # yaw angle
        self.position_unit_vector = np.array([math.cos(math.radians(data.pose[2])), math.sin(math.radians(data.pose[2]))]) # yaw unit vector
        
    def head_callback(self, data):
        self.head = np.array([data.pose[0], data.pose[1]])
        self.head_yaw = data.pose[2] # yaw angle
        self.head_unit_vector = np.array([math.cos(math.radians(data.pose[2])), math.sin(math.radians(data.pose[2]))]) # yaw unit vector

    def speed_callback(self, data):
        self.speed = data.speed

    def parking_mission(self, goal=Goal):
        # goal.position[0] = x coordinate / goal.position[1] = y coordinate / goal.yaw = yaw angle (degree)
        parking_spot = goal.position
        
        if self.parking_index == 0:
            # Check if a car is stopped
            if goal.mode == PARKING_SPOT and self.speed == 0:
                self.parking_start_time = time.time()
                self.parking_index += 1

        elif self.parking_index == 1:
            # After a car stopped
            if time.time() - self.parking_start_time >= 3:
                # Parking success
                rospy.loginfo("Parking mission success!")
                self.parking_flag = 1
                self.parking_index += 1
                self.parking_success = True

            elif time.time() - self.parking_start_time < 3 and self.speed != 0:
                # Parking fail
                rospy.loginfo("Parking mission failed...")
                self.parking_flag = 0
                self.parking_index = 0
                self.parking_start = False

                if not self.parking_updated:
                    # Spawn index
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)
                    self.parking_updated = True
        
            else:
                rospy.loginfo("Trying to park...")
            
        elif self.parking_index == 2:
            # parking misison failed, break the function
            if parking_spot is None:
                self.parking_start = False
            # parking mission success
            else:
                self.parking_index += 1
    
        elif self.parking_index == 3:
            if self.parking_success:
                self.num_success_parking[goal.number - 1] += self.parking_flag
                self.parked_spots.append(parking_spot)
            
            if not self.parking_updated:
                # Spawn index
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)

            # Reset the trigger
            self.parking_start = False
            self.parking_flag = 0
            self.parking_success = False
            self.parking_updated = False
            self.parking_index = 0

    #### left time and stop publish
    def stop_mission(self, goal=Goal):
        stop_spot = goal.position
        traffic_msg = Traffic()
        
        if self.stop_index == 0:
            # check if a car is stopped
            if goal.mode == STOP_LINE and self.speed == 0:
                rospy.loginfo("Stopped...")
                self.stop_time = time.time()
                self.stop_index += 1

            traffic_msg.traffic = "STOP"
            traffic_msg.second = param.STOP_LINE_TIME
            self.traffic.publish(traffic_msg)
        
        elif self.stop_index == 1:
            # After a car stopped
            if time.time() - self.stop_time >= param.STOP_LINE_TIME:
                # Stop success
                rospy.loginfo("Stop mission success!")
                self.stop_flag = 1
                self.stop_index += 1
                self.stop_success = True

            elif time.time() - self.stop_time < param.STOP_LINE_TIME and self.speed != 0:
                # Stop fail
                rospy.loginfo("Stop mission failed...")
                self.stop_flag = 0
                self.stop_index += 1
                self.stop_start = False

                if not self.stop_updated:
                    # Spawn index
                    complete_msg = Complete()
                    complete_msg.complete = True
                    self.complete.publish(complete_msg)
                    self.stop_updated = True
            
            else:
                rospy.loginfo("Trying to stop...")
            
            traffic_msg.traffic = "STOP"
            traffic_msg.second = param.STOP_LINE_TIME - (time.time() - self.stop_time)
            self.traffic.publish(traffic_msg)

        elif self.stop_index == 2:
            # stop mission failed, break the function
            if stop_spot is None:
                self.stop_start = False
                self.stop_index = 0
            # stop misison success
            else:
                self.stop_index += 1
        
        elif self.stop_index == 3:
            # Stop mission finally end
            self.num_success_stop[goal.number - 1] += self.stop_flag
            if self.stop_success:
                self.stopped_spots.append(stop_spot)

            if not self.stop_updated:
                # Spawn index
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)
            
            # Reset the trigger
            self.stop_start = False
            self.stop_flag = 0
            self.stop_success = False
            self.stop_updated = False
            self.stop_index = 0

    # Check if traffic mission is end
    def traffic_mission(self, goal=Goal):
        # goal.position[0] = x coordinate / goal.position[1] = y coordinate / goal.yaw = yaw angle (degree)
        traffic_spot = goal.position
        traffic_msg = Traffic()
        
        if self.traffic_index == 0:
            # Check if a car is stopped
            if goal.mode == STOP_LINE and self.speed == 0:
                rospy.loginfo("Stopped...")
                self.traffic_start_time = time.time()
                self.traffic_index += 1

            traffic_msg.traffic = "STOP"
            traffic_msg.second = param.STOP_LINE_TIME
            self.traffic.publish(traffic_msg)

        elif self.traffic_index == 1:
            # After a car stopped
            if time.time() - self.traffic_start_time >= param.STOP_LINE_TIME:
                # Traffic stop success
                rospy.loginfo("Stop mission success!")
                self.traffic_index += 1
                self.stop_flag = 1
                self.num_success_stop[goal.number - 1] += self.stop_flag

                # publish that parking mission is succeed
                complete_msg = Complete()
                complete_msg.complete = True
                self.complete.publish(complete_msg)

            elif time.time() - self.traffic_start_time < param.STOP_LINE_TIME and self.speed != 0:
                # Traffic fail
                rospy.loginfo("Stop mission failed...")
                self.traffic_index = 0
                self.stop_flag = 0

            else:
                rospy.loginfo("Trying to stop...")
            
            traffic_msg.traffic = "STOP"
            traffic_msg.second = param.STOP_LINE_TIME - (time.time() - self.traffic_start_time)
            self.traffic.publish(traffic_msg)
        
        elif self.traffic_index == 2:
            # traffic misison failed, break the function
            if traffic_spot is None:
                self.traffic_start = False
            # traffic mission success
            else:
                self.traffic_index += 1
    
        elif self.traffic_index == 3:
            # Check if a car is passed the correct road
            left_pos = np.array((param.MAP_2_CHECK_LEFT_X, param.MAP_2_CHECK_LEFT_Y))
            right_pos = np.array((param.MAP_2_CHECK_RIGHT_X, param.MAP_2_CHECK_RIGHT_Y))
            current_pos = None
            if dis(self.head, left_pos) < 0.5:
                current_pos = "LEFT"
            elif dis(self.head, right_pos) < 0.5:
                current_pos = "RIGHT"
            if current_pos is not None:
                if param.map_2_traffic_dir == current_pos:
                    rospy.loginfo("Correct Way!")
                    self.traffic_success = True
                    self.traffic_flag = 1
                else:
                    rospy.loginfo("Wrong Way...")
                    self.traffic_success = False
                    self.traffic_flag = 0
                
                self.traffic_index += 1

            traffic_msg.traffic = param.map_2_traffic_dir
            self.traffic.publish(traffic_msg)

        elif self.traffic_index == 4:
            if self.traffic_success:
                self.passed_traffic.append(traffic_spot)

            self.num_passed_traffic += 1
            self.num_success_traffic += self.traffic_flag

            # publish that parking mission is succeed
            complete_msg = Complete()
            complete_msg.complete = True
            self.complete.publish(complete_msg)

            # Reset the trigger
            self.traffic_start = False
            self.traffic_flag = 0
            self.traffic_success = False

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
                dist = np.linalg.norm(position_diff)

                left_dis = dis(self.head, np.array((param.MAP_2_CHECK_LEFT_X, param.MAP_2_CHECK_LEFT_Y)))
                right_dis = dis(self.head, np.array((param.MAP_2_CHECK_RIGHT_X, param.MAP_2_CHECK_RIGHT_Y)))

                # print(unit_vector_diff)
                # Stop
                if np.dot(self.head_unit_vector, -goal.unit_vector)<=0\
                    and np.dot(position_diff, -goal.unit_vector)<=0\
                    and dist <= goal.tolarance[0]:
                    check_flag = 1

                # Check if a car is in check area
                if left_dis <= 0.5 or right_dis <= 0.5:
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
        
    def publish_traffic(self, what):
        traffic = Traffic()
        traffic.traffic = what
        self.traffic.publish(traffic)

    def init_mission_button(self):
        # Refresh button
        window = tk.Tk()
        window.geometry('70x50+130+350')
        button = tk.Button(window, text='Mission\nInit', command=self.initialize)
        button.config(width=5, height=2)
        button.place(x=0, y=0)
        window.mainloop()

    def initialize(self):
        # refresh all the flags
        self.num_success_parking = [0,0]
        self.num_success_stop = [0,0]
        self.num_success_traffic = 0
        self.num_passed_traffic = 0
        self.parking_start = False
        self.stop_start = False
        self.traffic_start = False
        self.stop_index = 0
        self.parking_index = 0
        self.traffic_index = 0
        self.parking_updated = False
        self.stop_updated = False

    def visualize_mission(self):
        # Show the status of missions
        img = np.full(shape=(200,400,3),fill_value=0,dtype=np.uint8)
        if self.map_number == 1:
            cv2.imshow("mission_image",img)
            cv2.moveWindow("mission_image", 0, 150)
            cv2.waitKey(1)

        elif self.map_number == 2:
            text_1 = "Stop : "
            text_2 = "Traffic : "

            if self.num_success_stop[0] == 1:
                text_1 += "Success"
            
            if self.num_success_traffic == 1:
                text_2 += "Success"

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
            cv2.imshow("mission_image",img)
            cv2.moveWindow("mission_image", 0, 150)
            cv2.waitKey(1)

        elif self.map_number == 3:
            text_1 = "Parking #1 : "
            text_2 = "Parking #2 : "

            if self.num_success_parking[0] == 1:
                text_1 += "Success"
            if self.num_success_parking[1] == 1:
                text_2 += "Success"

            text_3 = "Stop #1 : "
            text_4 = "Stop #2 : "

            if self.num_success_stop[0] == 1:
                text_3 += "Success"
            if self.num_success_stop[1] == 1:
                text_4 += "Success"

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.putText(img, text_1, (0, 30), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_2, (0, 70), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_3, (0, 110), font, 1, (255, 255, 255), 2)
            cv2.putText(img, text_4, (0, 150), font, 1, (255, 255, 255), 2)
            cv2.imshow("mission_image",img)
            cv2.moveWindow("mission_image", 0, 150)
            cv2.waitKey(1)

if __name__ == "__main__":
    try:
        test_mission = Mission()
        rate = rospy.Rate(param.thread_rate)

        while not rospy.is_shutdown():
            try:
                test_mission.main()
                rate.sleep()
            except :
                continue
        
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")