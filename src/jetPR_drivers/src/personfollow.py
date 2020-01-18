#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('jetPR_drivers')
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.detectsub = rospy.Subscriber("detectnet/detections",Detection2DArray,self.dcbk)
    self.servopub = rospy.Publisher("servo",UInt16,queue_size = 10)
    self.vel_pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
    self.image_sub = rospy.Subscriber("imagee",Image,self.callback)
    self.cmd_vel = Twist()
    self.cv_image = np.zeros((640,480,3), np.uint8)
    self.final_img = np.zeros((640,480,3), np.uint8)
    self.servo = UInt16()
    self.flag = 0

  def dcbk(self,msg):
    for det in msg.detections:
      if det.results[0].id == 1 and det.results[0].score > 0.5 :
        if det.bbox.size_y == 400 and self.servo.data > 0:
          self.servo.data -= 1
        elif det.bbox.center.y > 240 and self.servo.data < 180:
          self.servo.data += 1
	elif det.bbox.center.y <240 and self.servo.data > 0:
          self.servo.data -= 1
        self.cmd_vel.angular.z = -0.2*(det.bbox.center.x - 320)/80
	area = det.bbox.size_x*det.bbox.size_y
        self.cmd_vel.linear.x = 0.1*(area - 76800)/76800
      else:
        self.cmd_vel.angular.z = 0
      self.vel_pub.publish(self.cmd_vel)
      self.servopub.publish(self.servo)

  def callback(self,data):
    if self.flag == 0:
       self.servo.data = 20
       self.servopub.publish(self.servo)
       self.flag = 1
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
