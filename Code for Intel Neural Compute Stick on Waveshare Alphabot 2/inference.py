#!/usr/bin/env python3

import cv2 as cv
import glob

# Load the model.
net = cv.dnn.readNet('image-classification-0002.xml', 'image-classification-0002.bin')

# Specify target device.
net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD)

# Read an image.
print('Loading blocked image')
frame = cv.imread('blocked.jpg')

# Get the list of images
images = glob.glob('images/*.jpg')

for image in images:
	# Read an image.
	print('Loading: '+image)
	frame = cv.imread(image)

	# Prepare input blob and perform an inference.
	blob = cv.dnn.blobFromImage(frame, size=(224, 224), ddepth=cv.CV_8U)
	net.setInput(blob)
	out = net.forward()

	for detection in out:
		print('clear:  \t'+str(detection[0]))
		print('blocked:\t'+str(detection[1]))