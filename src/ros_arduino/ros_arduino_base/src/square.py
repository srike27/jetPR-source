#!/usr/bin/env python
import sys
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist

def runner():
  vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
  cmd_vel = Twist()
  while(True):
    for i in range(4):
      cmd_vel.linear.x = 0.1
      cmd_vel.angular.z = 0
      vel_pub.publish(cmd_vel)
      time.sleep(1)
      cmd_vel.linear.x = 0
      cmd_vel.angular.z = 0
      vel_pub.publish(cmd_vel)
      time.sleep(1)
      cmd_vel.linear.x = 0.0
      cmd_vel.angular.z = 0.2
      vel_pub.publish(cmd_vel)
      time.sleep(7.853)
      cmd_vel.linear.x = 0
      cmd_vel.angular.z = 0
      vel_pub.publish(cmd_vel)
      time.sleep(1)
    time.sleep(5)

def main(args):
  rospy.init_node('square', anonymous=True)
  try:
    runner()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
