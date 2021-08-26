import cv2
import numpy as np
import math
import pathlib

def deg_to_rad(num):
	return num * 0.01745329251
def rad_to_deg(num):
	return num / 0.01745329251


# data = list(pathlib.Path("../../ml/green/data").glob("*/*.jpg"))
# err = 0
# max_count = 500
# counter = 0
# for filename in data:
# 	if(counter == max_count):
# 		break
image = cv2.imread("test_images/7.jpg")
#image = cv2.imread(filename.__str__())
image_debug = image.copy()
#image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

image = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)
image = cv2.medianBlur(image, 3)

