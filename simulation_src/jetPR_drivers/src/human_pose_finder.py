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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math as m

class pose_getter:

    def __init__(self):
        self.detectsub = rospy.Subscriber("objects",Detection2DArray,self.object_callback)
        self.measurepub = rospy.Publisher("human_pose_measured",PoseStamped,queue_size=20)
        self.bridge = CvBridge()
        self.hfov = 1.047198
        self.vfov = 3.0 * self.hfov / 4.0
        self.image_sub = rospy.Subscriber("camera/depth/image_raw",Image,self.Depthcallback)
    
    def object_callback(self,msg):
        for det in msg.detections:
            if det.results[0].id == 1 and det.results[0].score > 0.5 :
                px = int((4.0/3.0)*det.bbox.center.x)
                py = int((3.0/4.0)*det.bbox.center.y)
                sx = int((4.0/3.0)*det.bbox.size_x)
                sy = int((3.0/4.0)*det.bbox.size_y)
                img = self.d_image[py - sy/2:py + sy/2,px - sx/2:px + sx/2]
                res = np.where(img == np.nanmin(img))
                px = px - sx/2 + res[0][0]
                py = py - sy/2 + res[1][0]
                pxbar = px - 320
                pybar = 240 - py
                try:
                    d = self.d_image[py][px]
                    theta = m.atan(pybar*m.tan(self.vfov/2)/(240))
                    phi = m.atan(pxbar*m.tan(self.hfov/2)/(320))
                    self.x = d/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + ((m.tan(phi))*(m.tan(phi))))
                    self.y = -(d * m.tan(phi))/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + (m.tan(phi)*(m.tan(phi))))
                    self.z = (d * m.tan(theta))/m.sqrt(1/((m.cos(theta))*(m.cos(theta))) + (m.tan(phi)*(m.tan(phi))))
                    p = PoseStamped()
                    p.header.frame_id = 'camera_link'
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = self.x
                    p.pose.position.y = self.y
                    p.pose.position.z = self.z
                    p.pose.orientation.w = 1
                    self.measurepub.publish(p)
                except:
                    print("depth image not received")


    def Depthcallback(self,msg_depth):
        try:
            self.d_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = pose_getter()
    rospy.init_node('human_pose_finder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

