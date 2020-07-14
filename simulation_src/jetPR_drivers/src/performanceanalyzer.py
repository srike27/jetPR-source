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
import tf
import math as m
import csv

class pose_getter:

    def __init__(self):
        rospy.Subscriber("goal_pose", PoseStamped, self.callback)
        self.darray = []
        self.tflistener = tf.TransformListener()
    
    def callback(self,msg):
        self.x_est = msg.pose.position.x # from filter
        self.y_est = msg.pose.position.y
        try:
            (trans,rot) = self.tflistener.lookupTransform('/base_link', '/person_standing', rospy.Time(0))
            self.x_gt = trans[0] # from ground truth
            self.y_gt = trans[1]
            d = m.sqrt((self.x_est - self.x_gt)**2 + (self.y_est - self.y_gt)**2)
            self.darray.append(d)
            print(self.x_est,self.y_est,self.x_gt,self.y_gt,d)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        

    def listener(self):
        rate = rospy.Rate(60)
        while(True):
            rate.sleep()

    

def main(args):
    rospy.init_node('perfanalyze', anonymous=True)
    ic = pose_getter()
    try:
        ic.listener()
    except KeyboardInterrupt:
        with open('performancemetrics.csv', mode='w') as file:
            writer = csv.writer(file)
            sum = 0
            circle = 0.5
            consec = 0
            mconsec = 0
            failure = 0
            for i in ic.darray:
                sum += i*i
                if i > circle:
                    consec = 0
                    failure += 1
                else:
                    consec += 1
                    if consec > mconsec:
                        mconsec = consec
                writer.writerow(str(i))
            sum /= len(ic.darray)
            RMSE = m.sqrt(sum)
            frate = failure / len(ic.darray)
            print(RMSE,frate,mconsec)
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)

