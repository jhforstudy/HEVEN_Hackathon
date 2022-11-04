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

        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.id = 2
        traffic_stop.type = Marker.ARROW
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 0, 0, 1
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 1
        traffic_stop.scale.y = 1
        traffic_stop.scale.z = 0
        traffic_stop.pose.position.x = -3.96
        traffic_stop.pose.position.y =-18.3
        traffic_stop.pose.position.z = 0
        traffic_stop.pose.orientation.x = 0.0
        traffic_stop.pose.orientation.y = 0.0
        traffic_stop.pose.orientation.z = 0.0
        traffic_stop.pose.orientation.w = 1.0

        traffic_left = Marker()
        traffic_left.header.frame_id = "map"
        traffic_left.ns = "traffic_left"
        traffic_left.id = 3
        traffic_left.type = Marker.ARROW
        traffic_left.action = Marker.ADD
        traffic_left.color.r, traffic_left.color.g, traffic_left.color.b = 0, 0, 1
        traffic_left.color.a = 1
        traffic_left.scale.x = 1
        traffic_left.scale.y = 1
        traffic_left.scale.z = 0
        traffic_left.pose.position.x = -5.0
        traffic_left.pose.position.y = -17.1
        traffic_left.pose.position.z = 0
        traffic_left.pose.orientation.x = 0.0
        traffic_left.pose.orientation.y = 0.0
        traffic_left.pose.orientation.z = 1.0
        traffic_left.pose.orientation.w = 0.0

        return m

    elif map_number == 3:
        
        return None

    else:
        rospy.loginfo("Map number is incorrect.")

def MakeTrafficMarker(map_number):
    if map_number == 1:
        return None
    
    elif map_number == 2:
        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.id = 2
        traffic_stop.type = Marker.ARROW
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 0, 0, 1
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 1
        traffic_stop.scale.y = 0.1
        traffic_stop.scale.z = 0.1
        traffic_stop.pose.position.x = -3.96
        traffic_stop.pose.position.y =-18.3
        traffic_stop.pose.position.z = 0
        traffic_stop.pose.orientation.x = 0.0
        traffic_stop.pose.orientation.y = 0.0
        traffic_stop.pose.orientation.z = 0.7071068
        traffic_stop.pose.orientation.w = 0.7071068

        if param.map_2_traffic_dir == "LEFT":
            traffic_left = Marker()
            traffic_left.header.frame_id = "map"
            traffic_left.ns = "traffic_left"
            traffic_left.id = 3
            traffic_left.type = Marker.ARROW
            traffic_left.action = Marker.ADD
            traffic_left.color.r, traffic_left.color.g, traffic_left.color.b = 0, 0, 1
            traffic_left.color.a = 1
            traffic_left.scale.x = 1
            traffic_left.scale.y = 0.1
            traffic_left.scale.z = 0.1
            traffic_left.pose.position.x = -5.0
            traffic_left.pose.position.y = -17.1
            traffic_left.pose.position.z = 0
            traffic_left.pose.orientation.x = 0.0
            traffic_left.pose.orientation.y = 0.0
            traffic_left.pose.orientation.z = 1.0
            traffic_left.pose.orientation.w = 0.0

            return traffic_stop, traffic_left    

        elif param.map_2_traffic_dir == "RIGHT":
            traffic_right = Marker()
            traffic_right.header.frame_id = "map"
            traffic_right.ns = "traffic_right"
            traffic_right.id = 3
            traffic_right.type = Marker.ARROW
            traffic_right.action = Marker.ADD
            traffic_right.color.r, traffic_right.color.g, traffic_right.color.b = 0, 0, 1
            traffic_right.color.a = 1
            traffic_right.scale.x = 1
            traffic_right.scale.y = 0.1
            traffic_right.scale.z = 0.1
            traffic_right.pose.position.x = -3.2
            traffic_right.pose.position.y = -17.1
            traffic_right.pose.position.z = 0
            traffic_right.pose.orientation.x = 0.0
            traffic_right.pose.orientation.y = 0.0
            traffic_right.pose.orientation.z = 0.0
            traffic_right.pose.orientation.w = 1.0

            return traffic_stop, traffic_right
        
        else:
            rospy.loginfo("Traffic direction is incorrect.")

    elif map_number == 3:
        
        return None

    else:
        rospy.loginfo("Map number is incorrect.")

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker_array', MarkerArray, queue_size=1)
    rate = rospy.Rate(1)
    # Get mission number
    map_number = rospy.get_param('~map_number')

    goal_marker = MakeGoalMarker(map_number)
    traffic_marker = MakeTrafficMarker(map_number)

    while not rospy.is_shutdown():

        mkarray_msg = MarkerArray()
        temp_list = []
        temp_list.append(goal_marker)
        for i in traffic_marker:
            temp_list.append(i)
        mkarray_msg = temp_list

        visual_pub.publish(mkarray_msg)
        rate.sleep()
