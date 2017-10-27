
import sys
import serial

import time

try:
     ser = serial.Serial('/dev/ttyACM1',115200,timeout=1)
except serial.SerialException:
    try:
	ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
    except serial.SerialException:
        print (" You either aren't connected to port or see whether you used the correct open port or some port error")

v = 0
s = 0

while(True):
	read = ser.readline()
	string_status = read#.split(',')#ser.read(4)
	if(string_status[0] == "v"):
		v = 10*int(string_status[2])+int(string_status[3])
	elif(string_status[0] == "s"):
		if(string_status[3] == "1"):
			s = (10*int(string_status[1])+int(string_status[2]))
		elif(string_status[3] == "0"):
			s = (10*int(string_status[1])+int(string_status[2]))*(-1)
	file = open("status_can_.txt",'w') 
	file.write(str(v)+"\n")
	file.write(str(s)+"\n")
	print v
	print s
	file.close()
