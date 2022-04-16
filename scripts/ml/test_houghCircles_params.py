import cv2
import os
from os import listdir
import numpy as np

path = "/data/Programmieren/Robocup/2022/Main Repo 2022/robocup/ml_data/victims/victims_back_cam"

# read images and store grayscale version in list
img_paths = []
for images in os.listdir(path): 
	if (images.endswith(".jpg")):
		img_paths.append(path + '/' + images)
images = []
for img_path in img_paths:
	img = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2GRAY)
	img = img[170:479, 0:639]
	img = cv2.GaussianBlur(img, (7, 7), 0, 0)
	images.append(img)
for img in images:
	circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, dp=1.0, minDist=60, param1=54, param2=27, minRadius=28, maxRadius=305)
	output = img
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
			cv2.circle(output, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
	# show the output image
	cv2.imshow("output", output)
	cv2.waitKey(0)
