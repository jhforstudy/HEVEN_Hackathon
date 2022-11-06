#!/usr/bin/env python

import rospy

from math import pi
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from parameter_list import Param
from tf.transformations import quaternion_from_euler

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
        m = param.m
        m.pose.position.x = param.END_POINT_X_3
        m.pose.position.y = param.END_POINT_Y_3
        m.pose.position.z = 0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0

        return m

    else:
        rospy.loginfo("Map number is incorrect.")

def MakeTrafficMarker(map_number):
    if map_number == 1:
        return None
    
    elif map_number == 2:
        traffic_stop = Marker()
        traffic_stop.header.frame_id = "map"
        traffic_stop.ns = "traffic_stop"
        traffic_stop.type = Marker.LINE_STRIP
        traffic_stop.action = Marker.ADD
        traffic_stop.color.r, traffic_stop.color.g, traffic_stop.color.b = 1, 0, 0
        traffic_stop.color.a = 1
        traffic_stop.scale.x = 0.1
        traffic_stop.scale.y = 0.1
        traffic_stop.scale.z = 0
        l_point = Point()
        l_point.x = -4.60
        l_point.y = -16.45
        l_point.z = 0 
        r_point = Point()
        r_point.x = -3.39
        r_point.y = -16.45
        r_point.z = 0

        traffic_stop.points.append(l_point)
        traffic_stop.points.append(r_point)

        stop_sign = Marker()
        stop_sign.header.frame_id = "map"
        stop_sign.ns = "stop_sign"
        stop_sign.type = Marker.CYLINDER
        stop_sign.action = Marker.ADD
        stop_sign.color.r, stop_sign.color.g, stop_sign.color.b = 1, 0, 0
        stop_sign.color.a = 1
        stop_sign.scale.x = param.SIZE_OF_TROPHY
        stop_sign.scale.y = param.SIZE_OF_TROPHY
        stop_sign.scale.z = 0
        stop_sign.pose.position.x = -3.96
        stop_sign.pose.position.y = -16.9
        stop_sign.pose.position.z = 0
        stop_sign.pose.orientation.x = 0.0
        stop_sign.pose.orientation.y = 0.0
        stop_sign.pose.orientation.z = 0.0
        stop_sign.pose.orientation.w = 1.0

        if param.map_2_traffic_dir == "LEFT":
            traffic_left = Marker()
            traffic_left.header.frame_id = "map"
            traffic_left.ns = "traffic_left"
            traffic_left.type = Marker.ARROW
            traffic_left.action = Marker.ADD
            traffic_left.color.r, traffic_left.color.g, traffic_left.color.b = 0, 0, 1
            traffic_left.color.a = 1
            traffic_left.scale.x = 1
            traffic_left.scale.y = 0.1
            traffic_left.scale.z = 0.1
            traffic_left.pose.position.x = -5.00
            traffic_left.pose.position.y = -15.8
            traffic_left.pose.position.z = 0
            traffic_left.pose.orientation.x = 0.0
            traffic_left.pose.orientation.y = 0.0
            traffic_left.pose.orientation.z = 1.0
            traffic_left.pose.orientation.w = 0.0

            return traffic_stop, traffic_left, stop_sign

        elif param.map_2_traffic_dir == "RIGHT":
            traffic_right = Marker()
            traffic_right.header.frame_id = "map"
            traffic_right.ns = "traffic_right"
            traffic_right.type = Marker.ARROW
            traffic_right.action = Marker.ADD
            traffic_right.color.r, traffic_right.color.g, traffic_right.color.b = 0, 0, 1
            traffic_right.color.a = 1
            traffic_right.scale.x = 1
            traffic_right.scale.y = 0.1
            traffic_right.scale.z = 0.1
            traffic_right.pose.position.x = -2.91
            traffic_right.pose.position.y = -15.87
            traffic_right.pose.position.z = 0
            traffic_right.pose.orientation.x = 0.0
            traffic_right.pose.orientation.y = 0.0
            traffic_right.pose.orientation.z = 0.0
            traffic_right.pose.orientation.w = 1.0

            return traffic_stop, traffic_right, stop_sign
        
        else:
            rospy.loginfo("Traffic direction is incorrect.")

    elif map_number == 3:
        pass

    else:
        rospy.loginfo("Map number is incorrect.")

def ParkinglotMarker(m=Marker):
    m = Marker()
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.color.r, m.color.g, m.color.b = 0, 0, 1
    m.color.a = 1
    m.scale.x = param.PARKING_LOT_WIDTH
    m.scale.y = param.PARKING_LOT_HEIGHT
    m.scale.z = 0
    m.pose.position.z = 0
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0

    return m
    
def MakeParkinglotMarker():
    parking_lot_1 = ParkinglotMarker()
    parking_lot_1.header.frame_id = "map"
    parking_lot_1.ns = "lot_1"
    parking_lot_1.pose.position.x = param.PARKING_LOT_X_1
    parking_lot_1.pose.position.y = param.PARKING_LOT_Y_1

    parking_lot_2 = ParkinglotMarker()
    parking_lot_2.header.frame_id = "map"
    parking_lot_2.ns = "lot_2"
    parking_lot_2.pose.position.x = param.PARKING_LOT_X_2
    parking_lot_2.pose.position.y = param.PARKING_LOT_Y_2

    parking_lot_3 = ParkinglotMarker()
    parking_lot_3.header.frame_id = "map"
    parking_lot_3.ns = "lot_3"
    parking_lot_3.pose.position.x = param.PARKING_LOT_X_3
    parking_lot_3.pose.position.y = param.PARKING_LOT_Y_3

    parking_lot_4 = ParkinglotMarker()
    parking_lot_4.header.frame_id = "map"
    parking_lot_4.ns = "lot_4"
    parking_lot_4.pose.position.x = param.PARKING_LOT_X_4
    parking_lot_4.pose.position.y = param.PARKING_LOT_Y_4

    if param.map_3_parking_first_dir == 1:
        parking_lot_1.color.r, parking_lot_1.color.g, parking_lot_1.color.b = 0, 1, 0
    elif param.map_3_parking_first_dir == 2:
        parking_lot_2.color.r, parking_lot_2.color.g, parking_lot_2.color.b = 0, 1, 0
    elif param.map_3_parking_first_dir == 3:
        parking_lot_3.color.r, parking_lot_3.color.g, parking_lot_3.color.b = 0, 1, 0
    elif param.map_3_parking_first_dir == 4:
        parking_lot_4.color.r, parking_lot_4.color.g, parking_lot_4.color.b = 0, 1, 0

    tilt_degree = param.PARKING_LOT_TILT_DEGREE * pi / 180

    qu_x, qu_y, qu_z, qu_w = quaternion_from_euler(0,0,tilt_degree)

    parking_lot_5 = ParkinglotMarker()
    parking_lot_5.header.frame_id = "map"
    parking_lot_5.ns = "lot_5"
    parking_lot_5.pose.position.x = param.PARKING_LOT_X_5
    parking_lot_5.pose.position.y = param.PARKING_LOT_Y_5
    parking_lot_5.pose.orientation.x = qu_x
    parking_lot_5.pose.orientation.y = qu_y
    parking_lot_5.pose.orientation.z = qu_z
    parking_lot_5.pose.orientation.w = qu_w

    parking_lot_6 = ParkinglotMarker()
    parking_lot_6.header.frame_id = "map"
    parking_lot_6.ns = "lot_6"
    parking_lot_6.pose.position.x = param.PARKING_LOT_X_6
    parking_lot_6.pose.position.y = param.PARKING_LOT_Y_6
    parking_lot_6.pose.orientation.x = qu_x
    parking_lot_6.pose.orientation.y = qu_y
    parking_lot_6.pose.orientation.z = qu_z
    parking_lot_6.pose.orientation.w = qu_w

    parking_lot_7 = ParkinglotMarker()
    parking_lot_7.header.frame_id = "map"
    parking_lot_7.ns = "lot_7"
    parking_lot_7.pose.position.x = param.PARKING_LOT_X_7
    parking_lot_7.pose.position.y = param.PARKING_LOT_Y_7
    parking_lot_7.pose.orientation.x = qu_x
    parking_lot_7.pose.orientation.y = qu_y
    parking_lot_7.pose.orientation.z = qu_z
    parking_lot_7.pose.orientation.w = qu_w

    parking_lot_8 = ParkinglotMarker()
    parking_lot_8.header.frame_id = "map"
    parking_lot_8.ns = "lot_8"
    parking_lot_8.pose.position.x = param.PARKING_LOT_X_8
    parking_lot_8.pose.position.y = param.PARKING_LOT_Y_8
    parking_lot_8.pose.orientation.x = qu_x
    parking_lot_8.pose.orientation.y = qu_y
    parking_lot_8.pose.orientation.z = qu_z
    parking_lot_8.pose.orientation.w = qu_w

    if param.map_3_parking_second_dir == 5:
        parking_lot_5.color.r, parking_lot_5.color.g, parking_lot_5.color.b = 0, 1, 0
    elif param.map_3_parking_second_dir == 6:
        parking_lot_6.color.r, parking_lot_6.color.g, parking_lot_6.color.b = 0, 1, 0
    elif param.map_3_parking_second_dir == 7:
        parking_lot_7.color.r, parking_lot_7.color.g, parking_lot_7.color.b = 0, 1, 0
    elif param.map_3_parking_second_dir == 8:
        parking_lot_8.color.r, parking_lot_8.color.g, parking_lot_8.color.b = 0, 1, 0

    return parking_lot_1, parking_lot_2, parking_lot_3, parking_lot_4, parking_lot_5, parking_lot_6, parking_lot_7, parking_lot_8

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker_array', MarkerArray, queue_size=1)
    rate = rospy.Rate(param.thread_rate)
    # Get mission number
    map_number = rospy.get_param('~map_number')

    goal_marker = MakeGoalMarker(map_number)
    if map_number == 2:
        traffic_marker = MakeTrafficMarker(map_number)
    if map_number == 3:
        parking_marker = MakeParkinglotMarker()

    while not rospy.is_shutdown():

        mkarray_msg = MarkerArray()
        temp_list = []
        temp_list.append(goal_marker)
        if map_number == 2:
            for i in traffic_marker:
                temp_list.append(i)
        elif map_number == 3:
            for i in parking_marker:
                temp_list.append(i)
        mkarray_msg = temp_list

        visual_pub.publish(mkarray_msg)
        rate.sleep()
