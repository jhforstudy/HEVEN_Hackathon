#!/usr/bin/env python
import rospy
import tf
import math
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from racecar_simulator.msg import CenterPose, HeadPose


class PosePublisher():
    def __init__(self):
        rospy.init_node('pose_pub_node')
        # TF listener
        self.listener = tf.TransformListener()
        self.center_pub = rospy.Publisher("/car_center", CenterPose, queue_size=1)
        self.head_pub = rospy.Publisher("/car_head", HeadPose, queue_size=1)
        pass
    
    def main(self):
        # update pose info
        (trans_center,rot_center) = self.listener.lookupTransform('/map', '/center', rospy.Time(0))
        yaw_center = math.degrees((euler_from_quaternion(rot_center)[2]))
        center_data = [trans_center[0], trans_center[1], yaw_center]
        self.center_pub.publish(center_data)

        (trans_head,rot_head) = self.listener.lookupTransform('/map', '/head', rospy.Time(0))
        yaw_head = math.degrees((euler_from_quaternion(rot_head)[2]))
        head_data = [trans_head[0], trans_head[1], yaw_head]
        self.head_pub.publish(head_data)


if __name__ == "__main__":
    try:
        test_pose_pub = PosePublisher()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                test_pose_pub.main()
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    except rospy.ROSInterruptException:
        rospy.loginfo("Error!!!")