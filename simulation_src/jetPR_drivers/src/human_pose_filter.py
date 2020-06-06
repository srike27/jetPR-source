#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math as m
import numpy as np 
import time
import sys

class pose_filter:
    def __init__(self):
        self.goalpub = rospy.Publisher("goal_pose",PoseStamped,queue_size=20)
        self.measured_pose = rospy.Subscriber("human_pose_measured", PoseStamped, self.pose_callback)
        self.camera_tilt = rospy.Subscriber("camera_angle_controller/command", Float64, self.angle_callback)
        self.measure_flag = 0
        self.X = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.Z = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.time = time.clock()
        self.cam_angle = 0.0
        self.deltat = 0
        self.loophz = 20
        self.I = np.identity(6)
        self.A = np.array([[1,0,0,1/self.loophz,0,0],[0,1,0,0,1/self.loophz,0],[0,0,1,0,0,1/self.loophz],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.P = np.identity(6)
        self.Q = np.identity(6)
        self.R = np.identity(6)

    def angle_callback(self,msg):
        self.cam_angle = msg.data
    
    def pose_callback(self,msg):
        self.deltat = time.clock() - self.time
        self.Z[3] = (msg.pose.position.x - self.X[0])/self.deltat
        self.Z[4] = (msg.pose.position.y - self.X[1])/self.deltat
        self.Z[5] = (msg.pose.position.z - self.X[2])/self.deltat
        self.Z[0] = msg.pose.position.x
        self.Z[1] = msg.pose.position.y
        self.Z[2] = msg.pose.position.z
        self.time = time.clock()
        self.measure_flag = 1
    
    def listener(self):
        rate = rospy.Rate(self.loophz)
        while(True):
            if(self.measure_flag == 1):
                print("measurement rxed")
                Xnew = np.matmul(self.A,self.X)
                self.P = np.matmul(self.A,np.matmul(self.P,self.A.T)) + self.Q
                Y = self.Z - self.X
                K = np.matmul(self.P,np.linalg.inv(self.R + self.P))
                self.X = Xnew + np.matmul(K,Y)
                self.P = np.matmul((self.I-K),self.P)
                self.measure_flag = 0
            else:
                print("extrapolating from previous")
                Xnew = np.matmul(self.A,self.X)
                self.P = np.matmul(self.A,np.matmul(self.P,self.A.T)) + self.Q
                #Y = self.Z - self.X
                K = np.matmul(self.P,np.linalg.inv(self.R + self.P))
                self.X = Xnew
                self.P = np.matmul((self.I-K),self.P)
            try:
                dist_xz = m.sqrt(self.X[0]**2 + self.X[2]**2)
                ang_xz = m.atan(self.X[2]/self.X[0])
                x = dist_xz*m.cos(-self.cam_angle + ang_xz)
                z = dist_xz*m.sin(-self.cam_angle + ang_xz)
                y = self.X[1]
                p = PoseStamped()
                p.header.frame_id = 'camera_link'
                p.header.stamp = rospy.Time.now()
                p.pose.position.x = x + 0.035
                p.pose.position.y = y
                p.pose.position.z = z + 0.285
                p.pose.orientation.w = 1
                self.goalpub.publish(p)
            except:
                print("cam_angle not received")
            rate.sleep()

def main(args):
    rospy.init_node('human_pose_filter', anonymous=True)
    try:
        pf = pose_filter()
        pf.listener()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)
