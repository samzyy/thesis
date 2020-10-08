import serial
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import datetime
import math
import struct

left = 0
right = 0
framesPassedLeft = 0
framesPassedRight = 0

def handle_close(event):
	exit()

def tlvHeaderDecode(data):
	try:
		tlvType, tlvLength = struct.unpack('2I', data)
	except:
		return -1, - 1
	return tlvType, tlvLength

#9 cars passed in 20 senconds left lane

def filter_cars(objects):
	global left, right, framesPassedLeft, framesPassedRight
	counter = 0
	#Filter out stationary objects
	for i in range(len(objects)):
		if objects[counter][3] == 0:
			objects.pop(counter)
		else:
			counter += 1

	objects.sort(key=lambda x: x[0])
	leftLane = []
	rightLane = []
	for i in range(len(objects)):
		if objects[i][3] > 0:
			leftLane.append(objects[i])
		elif objects[i][3] < 0:
			rightLane.append(objects[i])
	countLeft = 0
	countRight = 0
	leftLane.sort(key=lambda x: x[1])
	for i in range(len(leftLane)):
		if leftLane[i][1] < 10:
			countLeft += 1
	for i in reversed(range(1, countLeft)):
		leftLane.pop(i)

	for i in range(len(rightLane)):
		if rightLane[i][1] < 10:
			countRight += 1
	for i in reversed(range(1, countRight)):
		rightLane.pop(i)
	
	objects = leftLane + rightLane
	display = objects
	for i in range(len(display)):
		display[i][0] = round(display[i][0], 2)
		display[i][1] = round(display[i][1], 2)
		display[i][2] = round(display[i][2], 2)
		display[i][3] = round(display[i][3], 2)
	print(display)

	for i in range(len(leftLane)):
		if leftLane[i][1] < 10:
			if left == 0:
				left += 1
				framesPassedLeft = 0
			elif framesPassedLeft > 15:
				left += 1
				framesPassedLeft = 0
	framesPassedLeft += 1

	for i in range(len(rightLane)):
		if rightLane[i][1] < 10:
			if right == 0:
				right += 1
				framesPassedRight = 0
			if framesPassedRight > 10:
				right += 1
				framesPassedRight = 0
	framesPassedRight += 1

	print("NUM OF LEFT CARS:", left)
	print("NUM OF RIGHT CARS:", right)
	
	return objects


def parseDetectedObjects(data, tlvLength, ax, timestamp):
	global left, right
	objects = []
	for i in range(0, tlvLength, 16):
		try:
			x, y, z, vel = struct.unpack('4f', data[i:i + 16])
			objects.append([x, y, z, vel])
		except:
			return
	ax.clear()
	ax.axis([-5, 5, 0, 10])
	# for n in filter_cars(objects):
	# 	if n[3] > 0:
	# 		ax.plot(n[0], n[1], 'bo', markersize=3)
	# 	if n[3] < 0:
	# 		ax.plot(n[0], n[1], 'ro', markersize=3)
	# plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
	# 		horizontalalignment='center', verticalalignment='center', 
	# 		transform=ax.transAxes)
	# plt.text(0.2, 0.7, "Cars pass in left lane: " + str(left), 
	# 		horizontalalignment='center', verticalalignment='center', 
	# 		transform=ax.transAxes)
	# plt.text(0.2, 0.6, "Cars pass in right lane: " + str(right), 
	# 		horizontalalignment='center', verticalalignment='center', 
	# 		transform=ax.transAxes)
	for n in objects:
		ax.plot(n[0], n[1], 'bo', markersize=3)
	plt.pause(0.001)
	plt.draw()

def tlvHeader(data, ax):
	counter = 0
	while data:
		headerLength = 36
		try:
			magic, version, length, platform, frameNum, cpuCycles, numObj, numTLVs = struct.unpack('Q7I', data[:headerLength])
		except:
			print("Improper TLV structure found: ", data)
			return
		counter += 1
		timestamp = str(datetime.timedelta(milliseconds=counter * 100))
		if len(timestamp) > 7:
			timestamp = timestamp[:9]
		else:
			timestamp = timestamp + ".0"
		if version > 0x01000005: 
			headerLength = 40
			pendingBytes = length - headerLength
			data = data[headerLength:]
			for i in range(numTLVs):
				tlvType, tlvLength = tlvHeaderDecode(data[:8])
				if tlvType == -1 and tlvLength == -1:
					return
				data = data[8:]
				if (tlvType == 1):
					parseDetectedObjects(data, tlvLength, ax, timestamp)
				data = data[tlvLength:]
				pendingBytes -= (8+tlvLength)
			data = data[pendingBytes:]

if __name__ == "__main__":
	configFile = sys.argv[1]
	# Windows
	try:
		CLIport = serial.Serial('COM4', 115200)
		dataPort = serial.Serial('COM3', 921600, timeout=0.1)
	except:
		print("Can't connect to serial ports.")
		exit()
	# Send configuration to device
	config = [line.rstrip('\r\n') for line in open(configFile)]
	for i in config:
		CLIport.write((i+'\n').encode())
		print(i)
		time.sleep(0.01)

	magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'

	plt.ion()
	fig, ax = plt.subplots()
	fig.canvas.mpl_connect('close_event', handle_close)
	buffer = bytearray()
	found = 0
	while 1:
		data = dataPort.read(32)
		if not(data):
			print("Error in serial connection. Please try again.")
			exit()
		offset = data.find(magic)
		if offset != -1:
			if found == 1:
				tlvHeader(bytes(buffer), ax)
				buffer = bytearray()
			else:
				found = 1
		if found:
			buffer.extend(data)


