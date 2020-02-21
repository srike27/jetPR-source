#!/usr/bin/env python
import serial
import time
import math

import rospy
import roslib
import tf
import tf2_ros

import geometry_msgs.msg
import std_msgs.msg

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform

if __name__== '__main__':
	rospy.init_node('map_odom',anonymous=True)
	while(True):
        	try:
            		#Broadcasting the transform
            		br = tf2_ros.TransformBroadcaster()
            		odom_tran = TransformStamped()
            		odom_quat = Quaternion()	
            		odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
            		odom_tran.header.frame_id = "odom"
			odom_tran.header.stamp = rospy.Time.now()
			odom_tran.child_frame_id = "map"
			odom_tran.transform.translation.x = 0.0
			odom_tran.transform.translation.y = 0.0
			odom_tran.transform.translation.z = 0.0
			odom_tran.transform.rotation.x = odom_quat[0]
			odom_tran.transform.rotation.y = odom_quat[1]
			odom_tran.transform.rotation.z = odom_quat[2]
			odom_tran.transform.rotation.w = odom_quat[3] 
			br.sendTransform(odom_tran)      
			    
        	except rospy.ROSInterruptException: #when you press ctrl+c
        	        print (rospy.ROSInterruptException)
                        pass
