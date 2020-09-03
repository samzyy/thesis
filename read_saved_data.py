import struct
import sys
import math
import datetime
import matplotlib.pyplot as plt
import numpy as np
import time

#Note: Press 'x' key to pause plot -> click any other key to unpause 
#or click 'x' again to step through

#Flag to pause
pause = 0
left = 0
framesPassed = 0

def handle_close(event):
    exit()

def press(event):
	global pause
	if event:
		pause = 1

def tlvHeaderDecode(data):
	tlvType, tlvLength = struct.unpack('2I', data)
	return tlvType, tlvLength

#9 cars passed in 20 senconds left lane

def filter_cars(objects):
	global left
	global framesPassed
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
		elif objects[i][0] < 0:
			rightLane.append(objects[i])
	count = 0
	leftLane.sort(key=lambda x: x[1])
	for i in range(len(leftLane)):
		if leftLane[i][1] < 10:
			count += 1
	for i in reversed(range(1, count)):
		leftLane.pop(i)
	
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
			if framesPassed == 0 or framesPassed > 10:
				left += 1
				framesPassed = 0
	framesPassed += 1

	print("NUM OF LEFT CARS:", left)
	

	return objects


def parseDetectedObjects(data, tlvLength, ax, timestamp):
	global pause
	global left
	objects = []
	for i in range(0, tlvLength, 16):
		x, y, z, vel = struct.unpack('4f', data[i:i + 16])
		objects.append([x, y, z, vel])
	ax.clear()
	ax.axis([-25, 25, 0, 50])
	for n in filter_cars(objects):
		ax.plot(n[0], n[1], 'bo', markersize=3)
	# for n in objects:
	# 	if n[3] != 0:
	# 		print("x =", n[0], "y =", n[1], "z =", n[2],"vel =", vel)
	# 		ax.plot(n[0], n[1], 'bo', markersize=3)
	plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
			horizontalalignment='center', verticalalignment='center', 
			transform=ax.transAxes)
	plt.text(0.2, 0.7, "Cars pass in left lane: " + str(left), 
			horizontalalignment='center', verticalalignment='center', 
			transform=ax.transAxes)
	plt.pause(0.1)
	plt.draw()
	#press(1)
	if pause == 1:
		pause = 0
		plt.waitforbuttonpress()

def tlvHeader(data, ax):
	counter = 0
	while data:
		headerLength = 36
		try:
			magic, version, length, platform, frameNum, cpuCycles, numObj, numTLVs = struct.unpack('Q7I', data[:headerLength])
		except:
			print("Improper TLV structure found: ", data)
			break
		counter += 1
		timestamp = str(datetime.timedelta(milliseconds=counter * 100))
		if len(timestamp) > 7:
			timestamp = timestamp[:9]
		else:
			timestamp = timestamp + ".0"
		print("Frame Number: ", counter)
		print("Timestamp:", timestamp)
		print("Detect Obj:\t%d "%(numObj))
		if version > 0x01000005: 
			headerLength = 40
			pendingBytes = length - headerLength
			data = data[headerLength:]
			for i in range(numTLVs):
				tlvType, tlvLength = tlvHeaderDecode(data[:8])
				data = data[8:]
				if (tlvType == 1):
					parseDetectedObjects(data, tlvLength, ax, timestamp)
				data = data[tlvLength:]
				pendingBytes -= (8+tlvLength)
			data = data[pendingBytes:]

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print("Usage: parseTLV.py inputFile.bin")
		sys.exit()

	#Open data file for reading
	fileName = sys.argv[1]
	rawDataFile = open(fileName, "rb")
	rawData = rawDataFile.read()
	rawDataFile.close()
	magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'
	offset = rawData.find(magic)
	rawData = rawData[offset:]

	#Generate plot
	plt.ion()
	fig, ax = plt.subplots()
	fig.canvas.mpl_connect('close_event', handle_close)
	fig.canvas.mpl_connect('key_press_event', press)

	tlvHeader(rawData, ax)
