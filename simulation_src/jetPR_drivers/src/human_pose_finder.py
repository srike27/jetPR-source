#!/usr/bin/env python
from __future__ import print_function
import numpy as np 
import roslib
roslib.load_manifest('jetPR_drivers')
import sys
import rospy
import cv2
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math as m

class pose_getter:

    def __init__(self):
        self.detectsub = rospy.Subscriber("objects",Detection2DArray,self.object_callback)
        self.bridge = CvBridge()
        self.hfov = 1.047198
        self.vfov = 3.0 * self.hfov / 4.0
        self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.Depthcallback)
    
    def object_callback(self,msg):
        for det in msg.detections:
            if det.results[0].id == 1 and det.results[0].score > 0.5 :
                px = int((4.0/3.0)*det.bbox.center.x)
                py = int((3.0/4.0)*det.bbox.center.y)
                pxbar = px - 320
                pybar = 240 - py
                #print(px,py)
                try:
                    d = self.d_image[py][px]
                    print(d)
                    theta = m.atan(pybar*m.tan(self.vfov/2)/(240))
                    phi = m.atan(pxbar*m.tan(self.hfov/2)/(320))
                    self.x = d/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + ((m.tan(phi))*(m.tan(phi))))
                    self.y = -(d * m.tan(phi))/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + (m.tan(phi)*(m.tan(phi))))
                    self.z = (d * m.tan(theta))/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + (m.tan(phi)*(m.tan(phi))))
                    print(self.x,self.y,self.z)
                except:
                    print("depth image not received")


    def Depthcallback(self,msg_depth):
        try:
            self.d_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
            #print(self.d_image.shape)
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = pose_getter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

