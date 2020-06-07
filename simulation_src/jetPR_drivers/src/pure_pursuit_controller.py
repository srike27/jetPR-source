#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import math as m

twpub = rospy.Publisher('cmd_vel',Twist,queue_size=10)
tw = Twist()

def callback(msg):
    maxv = 1.0
    close_dist = 1.0
    vx = 0.0
    x = msg.pose.position.x
    y = msg.pose.position.y
    dist = m.sqrt(x**2 + y**2)
    r = dist**2 / (2*y)
    if dist > 2:
        wicr = maxv/r
        wz = 2*wicr
        vx = maxv
    elif dist < 2:
        err = (dist - close_dist)
        vx = maxv*err/1.5
        wicr = vx/r
        wz = 2*wicr
    tw.linear.x = vx
    tw.angular.z = wz
    twpub.publish(tw)

def listener():
    rospy.init_node('pure_pursuit', anonymous=True)
    rospy.Subscriber("goal_pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()