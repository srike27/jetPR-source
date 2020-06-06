#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import math as m

angpub = rospy.Publisher('camera_angle_controller/command',Float64,queue_size=10)
ang = Float64()
servo_ang = 0

def callback(msg):
    global servo_ang
    z = msg.pose.position.z
    x = msg.pose.position.x
    cam_front = 0.035
    cam_height = 0.285
    theta = -m.tan((z-cam_height)/(x-cam_front))
    if theta - ang.data < -0.5:
        ang.data -= 0.01
    elif theta - ang.data > 0.5:
        ang.data += 0.01
    angpub.publish(ang)

def listener():
    rospy.init_node('servo_angle_controller', anonymous=True)
    rospy.Subscriber("goal_pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()