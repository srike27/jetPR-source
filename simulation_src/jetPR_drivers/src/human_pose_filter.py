#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math as m
import numpy as np 
import time
import sys

class pose_filter:
    def __init__(self):
        self.goalpub = rospy.Publisher("goal_pose",PoseStamped,queue_size=20)
        self.measured_pose = rospy.Subscriber("human_pose_measured", PoseStamped, self.pose_callback)
        self.measured_pose = rospy.Subscriber("odom", Odometry, self.odom_callback)
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
        self.d = 1.0
        self.wz = 0.0
        self.vx = 0.0
        self.u = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.flags = 0

    def odom_callback(self,msg):
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

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
            try:
                if self.flags == 1:
                    ux = -self.d*self.wz*self.y*m.sqrt(self.x**2 + self.z**2)/(self.x*m.sqrt(self.x**2+self.y**2)) - self.vx*m.sqrt(self.x**2 + self.z**2)/(m.sqrt((self.x**2+self.y**2)*(self.x**2 + self.z**2)))
                    uy = -self.d*self.wz*m.sqrt(self.x**2 + self.z**2)/(m.sqrt(self.x**2+self.y**2)) + self.vx*self.y*self.x/(self.x*m.sqrt(self.x**2+self.y**2))
                    self.u = np.array([0.0,0.0,0.0,ux,uy,0.0])
            except:
                print('Person not found')
            if(self.measure_flag == 1):
                print("measurement rxed")
                Xnew = np.matmul(self.A,self.X) + self.u
                self.P = np.matmul(self.A,np.matmul(self.P,self.A.T)) + self.Q
                Y = self.Z - self.X
                K = np.matmul(self.P,np.linalg.inv(self.R + self.P))
                self.X = Xnew + np.matmul(K,Y)
                self.P = np.matmul((self.I-K),self.P)
                self.measure_flag = 0
            else:
                print("extrapolating from previous")
                Xnew = np.matmul(self.A,self.X) + self.u
                self.P = np.matmul(self.A,np.matmul(self.P,self.A.T)) + self.Q
                #Y = self.Z - self.X
                K = np.matmul(self.P,np.linalg.inv(self.R + self.P))
                self.X = Xnew
                self.P = np.matmul((self.I-K),self.P)
            try:
                #print(self.Z)
                dist_xz = m.sqrt(self.X[0]**2 + self.X[2]**2)
                ang_xz = m.atan(self.X[2]/self.X[0])
                self.x = dist_xz*m.cos(-self.cam_angle + ang_xz)
                self.z = dist_xz*m.sin(-self.cam_angle + ang_xz)
                self.y = self.X[1]
                self.d = m.sqrt(self.x**2 + self.y**2 + self.z**2)
                if m.isnan(self.d) == False:
                    self.flags = 1
                p = PoseStamped()
                p.header.frame_id = 'camera_link'
                p.header.stamp = rospy.Time.now()
                p.pose.position.x = self.x + 0.035
                p.pose.position.y = self.y
                p.pose.position.z = self.z + 0.285
                p.pose.orientation.w = 1
                self.goalpub.publish(p)
            except:
                print("problem3")
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
