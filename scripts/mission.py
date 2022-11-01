#!/usr/bin/env python

import rospy
import time
import math
from goal import Goal

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

        # ROS settings
        rospy.init_node('mission', anonymous=True)
        rospy.Subscriber("/car_center", CenterPose, self.position_callback, queue_size=1)
        rospy.Subscriber("/car_head", HeadPose, self.head_callback, queue_size=1)
        self.traffic = rospy.Publisher("/traffic", Traffic, queue_size=1)

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

    def check(self):
        x_diff = Goal.x - self.position_x
        y_diff = Goal.y - self.position_y

        Goal.[x_diff, y_diff]

        Goal.yaw - self.position_yaw
    
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