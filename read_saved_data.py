import struct
import sys
import math
import datetime
import matplotlib.pyplot as plt
import numpy as np
import time

# Note: 
# In plot = 1 and step = 0 mode:
# Press any key to pause plot and click any key again to step through
# Press mouse button to unpause

# In plot = 1 and step = 1 mode:
# Press any key to step through the plot

# In plot = 0 mode:
# Program runs analysis much faster without plot
# Look at terminal for results

#Flags
pause = 0
left = 0
right = 0
framesPassedLeft = 0
framesPassedRight = 0
plot = 1
step = 0

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
	global pause, left, right
	objects = []
	for i in range(0, tlvLength, 16):
		x, y, z, vel = struct.unpack('4f', data[i:i + 16])
		objects.append([x, y, z, vel])
	if plot:
		ax.clear()
		ax.axis([-25, 25, 0, 50])
		for n in filter_cars(objects):
			if n[3] > 0:
				ax.plot(n[0], n[1], 'bo', markersize=3)
			if n[3] < 0:
				ax.plot(n[0], n[1], 'ro', markersize=3)
		plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax.transAxes)
		plt.text(0.2, 0.7, "Cars pass in left lane: " + str(left), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax.transAxes)
		plt.text(0.2, 0.6, "Cars pass in right lane: " + str(right), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax.transAxes)
		plt.pause(0.05)
		plt.draw()
	else:
		filter_cars(objects)
	if plot and step:
		press(1)
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

def argument_check(argv):
	global plot, step
	if not(len(argv) == 2 or len(argv) == 4):
		print("Usage: read_saved_data.py inputFile.bin --optional [plot step]")
		sys.exit()
	if len(argv) == 4:
		try:
			plot = int(argv[2])
			step = int(argv[3])
			if not((plot == 0 or plot == 1) and (step == 0 or step == 1)):
				raise ValueError()
		except Exception as e:
			print("Invalid plot or step value")
			sys.exit()

if __name__ == "__main__":
	argument_check(sys.argv)
	#Open data file for reading
	fileName = sys.argv[1]
	try:
		rawDataFile = open(fileName, "rb")
	except:
		print("Invalid data file")
		sys.exit()
	rawData = rawDataFile.read()
	rawDataFile.close()
	magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'
	offset = rawData.find(magic)
	rawData = rawData[offset:]

	#Generate plot
	if plot:
		plt.ion()
		fig, ax = plt.subplots()
		fig.canvas.mpl_connect('close_event', handle_close)
		fig.canvas.mpl_connect('key_press_event', press)
	else:
		#Dummy variable
		ax = 1

	tlvHeader(rawData, ax)
