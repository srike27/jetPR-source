#!/usr/bin/env python
import serial
import rospy
from std_msgs.msg import Float64


def main():
	ser = serial.Serial(port = '/dev/ttyTHS1', baudrate = 115200, timeout = 1, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
	pub = rospy.Publisher('voice_doa', Float64, queue_size=10)
	rospy.init_node('pi_rx')
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		while True:
			bytesToRead = ser.inWaiting()
			if(bytesToRead>0):
				print(ser.read(bytesToRead))
				doa = float(ser.read(bytesToRead))
				doangle = Float64()
				doangle.data = doa 
   				pub.publish(doangle)
   			r.sleep()

if __name__=="__main__":
	main()
	#except:
	#	print('Error!')
	#finally:
	#	print('Terminating connection with Raspberry pi')
