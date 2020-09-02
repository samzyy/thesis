import struct
import sys
import math
import datetime
import matplotlib.pyplot as plt
import numpy as np
import time
import threading

#Flags
pause = 0

def handle_close(event):
    exit()

def press(event):
	global pause
	if event.key == 'x':
		pause = not(pause)
		print("PAUSE")

def tlvHeaderDecode(data):
	tlvType, tlvLength = struct.unpack('2I', data)
	return tlvType, tlvLength

def parseDetectedObjects(data, tlvLength, ax, timestamp):
	objects = [[], [], [], []]
	for i in range(0, tlvLength, 16):
		x, y, z, vel = struct.unpack('4f', data[i:i + 16])
		objects[0].append(x)
		objects[1].append(y)
		objects[2].append(z)
		objects[3].append(vel)
		print("x =", x, "y =", y, "z =", z,"vel =", vel)
	ax.clear()
	ax.axis([-25, 25, 0, 50])
	ax.plot(objects[0], objects[1], 'bo', markersize=3)
	plt.text(0.5, 0.95, "Timestamp: " + str(timestamp), 
			horizontalalignment='center', verticalalignment='center', 
			transform=ax.transAxes)
	plt.pause(0.1)
	plt.draw()

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
