import serial
ser = serial.Serial(port = '/dev/ttyTHS1', baudrate = 115200, timeout = 1, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
while True:
	bytesToRead = ser.inWaiting()
	if(bytesToRead>0):
		print(ser.read(bytesToRead))
	
