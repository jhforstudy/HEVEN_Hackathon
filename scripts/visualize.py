#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from parameter_list import Param

param = Param()

def MakeGoalMarker(map_number):
    if map_number == 1:
        m = param.m
        m.pose.position.x = param.END_POINT_X_1
        m.pose.position.y = param.END_POINT_Y_1
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m
    
    elif map_number == 2:
        m = param.m
        m.pose.position.x = param.END_POINT_X_2
        m.pose.position.y = param.END_POINT_Y_2
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m

    elif map_number == 3:
        
        return None

    else:
        rospy.loginfo("Mission number is incorrect.")

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker', Marker, queue_size=1)
    rate = rospy.Rate(1)
    # Get mission number
    map_number = rospy.get_param('~map_number')

    goal_marker = MakeGoalMarker(map_number)

    while not rospy.is_shutdown():
        visual_pub.publish(goal_marker)
        rate.sleep()
