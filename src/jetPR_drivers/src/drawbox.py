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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("boxed",Image,queue_size = 10)

    self.bridge = CvBridge()
    self.detectsub = rospy.Subscriber("detectnet/detections",Detection2DArray,self.dcbk)
    self.image_sub = rospy.Subscriber("imagee",Image,self.callback)
    self.cv_image = np.zeros((640,480,3), np.uint8)
    self.final_img = np.zeros((640,480,3), np.uint8)

  def dcbk(self,msg):
    for det in msg.detections:
      for re in det.results:
        if re.id == 1 and re.score > 0.75 :
          pt1 = (int(det.bbox.center.x - det.bbox.size_x/2),int(det.bbox.center.y - det.bbox.size_y/2))
          pt2 = (int(det.bbox.center.x + det.bbox.size_x/2),int(det.bbox.center.y + det.bbox.size_y/2))
          self.final_img = cv2.rectangle(self.cv_image, pt1, pt2, color=(0, 255, 0), thickness=3)
        elif re.score > 0.75:
          pt1 = (int(det.bbox.center.x - det.bbox.size_x/2),int(det.bbox.center.y - det.bbox.size_y/2))
          pt2 = (int(det.bbox.center.x + det.bbox.size_x/2),int(det.bbox.center.y + det.bbox.size_y/2))
          self.final_img = cv2.rectangle(self.cv_image, pt1, pt2, color=(255, 0, 0), thickness=3)


  def callback(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.imshow('bounding_box',self.final_img)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.final_img, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
