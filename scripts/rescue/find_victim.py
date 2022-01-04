import math
import cv2
import numpy as np

img = cv2.imread("img/1.png")
size_x = 640
size_y = 480
MIN_Y = 40
MAX_Y = 220
MIN_CORNER_AREA = 100
MIN_CORNER_ASPECT_RATIO = 2 # width / height

# Cut out region of interest
roi = img[int(size_y * 0.35):size_y, 0:size_x]
roi_debug = roi.copy()

roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

# Blur
roi = cv2.GaussianBlur(roi, (7, 7), cv2.BORDER_DEFAULT)

# cv2.imshow("Blur", roi)
# cv2.waitKey(1000)

# circles = cv2.HoughCircles(roi, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=23, param2=55, maxRadius=300, minRadius=0)

# if circles is not None:
# 	circles = np.round(circles[0, :]).astype("int")

# 	for (x, y, r) in circles:
# 		cv2.circle(roi_debug, (x, y), r, (0, 255, 0), 3)

ret, black = cv2.threshold(roi, 50, 255, 1)

contours, hierarchy = cv2.findContours(black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
corner_contour = None

cv2.drawContours(roi_debug, contours, -1, (255, 0, 0), 3)

for c in contours:
	A = cv2.contourArea(c)
	x, y, w, h = cv2.boundingRect(c)
	if(y > MIN_Y and y < MAX_Y and A > MIN_CORNER_AREA and
		w / h > MIN_CORNER_ASPECT_RATIO):
		corner_contour = c
		break

if corner_contour is not None:
	cv2.drawContours(roi_debug, [c], 0, (0, 255, 0), 3)

cv2.imshow("ROI", roi_debug)
cv2.waitKey(1000)