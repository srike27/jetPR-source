#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
import math as m

angpub = rospy.Publisher('servo',Int16,queue_size=10)
ang = Int16()
servo_ang = 0

def callback(msg):
    global servo_ang
    z = msg.pose.position.z
    x = msg.pose.position.x
    cam_front = 0.035
    cam_height = 0.285
    theta = m.tan((z-cam_height)/(x-cam_front))
    ang.data = int(theta*180/m.pi)
    if(ang.data >  servo_ang):
        servo_ang += 1
    if(ang.data < servo_ang):
        servo_ang -= 1
    ang.data = servo_ang
    angpub.publish(ang)

def listener():
    rospy.init_node('servo_angle_controller', anonymous=True)
    rospy.Subscriber("goal_pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()