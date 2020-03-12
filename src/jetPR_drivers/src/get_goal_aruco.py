#!/usr/bin/env python  
import roslib
roslib.load_manifest('jetPR_drivers')
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('aruco_goal')

    listener = tf.TransformListener()
    posepub = rospy.Publisher('goal_pose', PoseStamped, queue_size=15)
    pose = PoseStamped()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform( 'base_link','aruco_marker_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.z = math.tan((rot[2])/2)
        pose.pose.orientation.w = 1
        n = pose.pose.orientation.z**2 + pose.pose.orientation.w**2
        pose.pose.orientation.z = pose.pose.orientation.z / n
        pose.pose.orientation.w = pose.pose.orientation.w / n
        posepub.publish(pose)
        rate.sleep()