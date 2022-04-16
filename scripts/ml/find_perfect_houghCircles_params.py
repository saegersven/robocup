import cv2
import os
from os import listdir
import numpy as np


path = "/mnt/D2F891CDF891AFE9/Programmieren/Robocup/2022/Main Repo 2022/robocup/ml_data/victims/victims_back_cam"
circles_in_images = 39 # how many circles should our program find?

def nr_circles(grayscale_img, dp, minDist, param1, param2, minRadius, maxRadius): 
	circles = cv2.HoughCircles(grayscale_img, cv2.HOUGH_GRADIENT, dp=dp, minDist=minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)
	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		return len(circles)
	return 0

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

# try lots of param combinations:
for dp in range(8, 12):
	dp = dp / 10 # in range using floats not possible
	print("DP:", dp)
	for param1 in range(20, 55):
		#print("	Param1:", param1)
		for param2 in range(20, 55):
			#print("		Param2:", param2)
			circles_cnt = 0
			for img in images:
				circles_cnt = circles_cnt + nr_circles(img, dp, 60, param1, param2, 28, 305)
				if circles_cnt > circles_in_images:
					break
			#print("			Circles found in images:", circles_cnt)
			if circles_cnt == circles_in_images:
				print("Possible param combination:", dp, 60, param1, param2, 28, 305)

"""
# Best param combinations for GaussianBlur(img, (7, 7), 0, 0): (for testing these refer to test_houghCircles_params.py)
DP: 0.8
Possible param combination: 0.8 60 38 31 28 305
Possible param combination: 0.8 60 54 27 28 305
DP: 1.0
Possible param combination: 1.0 60 54 27 28 305
DP: 1.1
Possible param combination: 1.1 60 28 39 28 305
Possible param combination: 1.1 60 42 37 28 305
Possible param combination: 1.1 60 54 32 28 305
"""
