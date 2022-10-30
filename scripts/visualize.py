#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

WHEELBASE = 0.425
REAR_LIDAR = 0.325
WIDTH = 0.145
SIZE_OF_TROPHY = 0.1

# Endpoint of Map 1
END_POINT_X = 13.34
END_POINT_Y = -9.31

from visualization_msgs.msg import Marker, MarkerArray

def MakeGoalMarker():
    m = Marker()

    m.header.frame_id = "map"
    m.ns = "goal_marker"
    m.id = 1
    m.type = Marker.POINTS
    m.action = Marker.ADD
    m.color.r, m.color.g, m.color.b = 1, 1, 0
    m.color.a = 1

    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.0

    m.points.append(Point(END_POINT_X, END_POINT_Y, 0))

    return m

if __name__ == "__main__":
    rospy.init_node("Visualize_node")
    visual_pub = rospy.Publisher('marker', Marker, queue_size=1)
    rate = rospy.Rate(1)
    goal_marker = MakeGoalMarker()

    while not rospy.is_shutdown():
        visual_pub.publish(goal_marker)
        rate.sleep()
