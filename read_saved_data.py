import struct
import sys
import math
import datetime
import matplotlib.pyplot as plt
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
	#Categorise vechicles into left and right lanes based on velocity
	for i in range(len(objects)):
		if objects[i][3] > 0:
			leftLane.append(objects[i])
		elif objects[i][3] < 0:
			rightLane.append(objects[i])
	countLeft = 0
	countRight = 0
	#Sort left and right lane by objects closest to the radar (y axis)
	leftLane.sort(key=lambda x: x[1])
	rightLane.sort(key=lambda x: x[1])
	#Count the number of objects detected close to the radar
	for i in range(len(leftLane)):
		if leftLane[i][1] < 10:
			countLeft += 1
	#Remove redundant detected objects close to radar (only need 1)
	for i in reversed(range(1, countLeft)):
		leftLane.pop(i)

	#Do the same for the right lane
	for i in range(len(rightLane)):
		if rightLane[i][1] < 10:
			countRight += 1
	for i in reversed(range(1, countRight)):
		rightLane.pop(i)
	
	objects = leftLane + rightLane
	#Printing detected objects (after removing redundant objects)
	display = objects
	print("Detect Obj (After Processing): {}".format(len(objects)))
	for i in range(len(display)):
		display[i][0] = round(display[i][0], 2)
		display[i][1] = round(display[i][1], 2)
		display[i][2] = round(display[i][2], 2)
		display[i][3] = round(display[i][3], 2)
	print(display)

	for i in range(len(leftLane)):
		if leftLane[i][1] < 10 and leftLane[i][0] < 3:
			if left == 0:
				left += 1
				framesPassedLeft = 0
			#Wait 15 frames for object to get out of y = 10 threshold
			elif framesPassedLeft > 15:
				left += 1
				framesPassedLeft = 0
	framesPassedLeft += 1

	for i in range(len(rightLane)):
		if rightLane[i][1] < 10:
			if right == 0:
				right += 1
				framesPassedRight = 0
			#Wait 10 frames for object to disappear out of y = 10 threshold
			if framesPassedRight > 10:
				right += 1
				framesPassedRight = 0
	framesPassedRight += 1

	print("NUM OF LEFT VEHICLES:", left)
	print("NUM OF RIGHT VEHICLES:", right)
	
	return objects

def parseDetectedObjects(data, tlvLength, ax, timestamp):
	global pause, left, right, framesPassedLeft, framesPassedRight
	objects = []
	for i in range(0, tlvLength, 16):
		x, y, z, vel = struct.unpack('4f', data[i:i + 16])
		objects.append([x, y, z, vel])
	if plot:
		ax[0].clear()
		ax[1].clear()
		ax[0].set_title("Original data")
		ax[1].set_title("Processed data")
		ax[0].axis([-25, 25, 0, 50])
		ax[1].axis([-10, 20, 0, 50])
		ax[0].set_xlabel("Distance along lateral axis (metres)")
		ax[1].set_xlabel("Distance along lateral axis (metres)")
		ax[0].set_ylabel("Distance along longitudinal axis (metres)")
		for n in objects:
			ax[0].plot(n[0], n[1], 'bo', markersize=3)
		for n in filter_cars(objects):
			if n[3] > 0:
				ax[1].plot(n[0], n[1], 'bo', markersize=3)
			if n[3] < 0:
				ax[1].plot(n[0], n[1], 'ro', markersize=3)
			ax[1].annotate("  {} km/h".format(n[3] * 3600 / 1000), (n[0], n[1]))
		plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax[1].transAxes)
		plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax[0].transAxes)
		plt.text(0.15, 0.8, "Cars passed in left lane: " + str(left), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax[1].transAxes)
		plt.text(0.155, 0.75, "Cars passed in right lane: " + str(right), 
				horizontalalignment='center', verticalalignment='center', 
				transform=ax[1].transAxes)
		plt.pause(0.1)
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
		print("Usage: read_saved_data.py data_file.dat --optional [plot step]")
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
	data = rawDataFile.read()
	rawDataFile.close()
	magic = b'\x02\x01\x04\x03\x06\x05\x08\x07'
	offset = data.find(magic)
	data = data[offset:]

	#Generate plot
	if plot:
		plt.ion()
		fig, ax = plt.subplots(1, 2, figsize=(15,8))
		plt.tight_layout()
		plt.gcf().subplots_adjust(bottom=0.07)
		plt.gcf().subplots_adjust(top=0.95)
		plt.gcf().subplots_adjust(right=0.95)
		plt.gcf().subplots_adjust(left=0.05)
		fig.canvas.mpl_connect('close_event', handle_close)
		fig.canvas.mpl_connect('key_press_event', press)
	else:
		#Dummy variable
		ax = 1

	tlvHeader(data, ax)
