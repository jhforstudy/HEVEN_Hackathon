#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from parameter_list import Param

param = Param()

def MakeGoalMarker(mission_number):
    if mission_number == 1:
        m = param.m
        m.points.append(Point(param.END_POINT_X_1, param.END_POINT_Y_1, 0))

        return m
    
    elif mission_number == 2:
        m = param.m
        m.points.append(Point(param.END_POINT_X_2, param.END_POINT_Y_2, 0))

        return m

    elif mission_number == 3:
        
        return None

    else:
        rospy.loginfo("Mission number is incorrect.")

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker', Marker, queue_size=1)
    rate = rospy.Rate(1)
    # Get mission number
    mission_number = rospy.get_param('~mission_number')

    goal_marker = MakeGoalMarker(mission_number)

    while not rospy.is_shutdown():
        visual_pub.publish(goal_marker)
        rate.sleep()
