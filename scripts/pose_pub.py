#!/usr/bin/env python
import rospy
import tf
import math
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from racecar_simulator.msg import CarPose


class PosePublisher():
    def __init__(self):
        rospy.init_node('pose_pub_node')
        # TF listener
        self.listener = tf.TransformListener()
        self.pose_pub = rospy.Publisher("/car_pose", CarPose, queue_size=1)
        pass
    
    def main(self):
        # update pose info
        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        yaw = math.degrees((euler_from_quaternion(rot)[2]))
        pose_data = [trans[0], trans[1], yaw]
        self.pose_pub.publish(pose_data)

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