import serial
import time
ser = serial.Serial(port = '/dev/ttyUSB0',baudrate = 115200, timeout = 1, parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)  # open first serial port
#print ser.portstr       # check which port was really used
while True:
	str = "hello"
	ser.write(str)
	time.sleep(0.01)
ser.close()             # close port