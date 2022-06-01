import cv2
import os
import numpy as np

GREEN_RATIO_THRESHOLD = 0.8
GREEN_MINIMUM_VALUE = 30

def is_green(b, g, r):
	return (float(g) / (float(b) + float(r)) > GREEN_RATIO_THRESHOLD and (float(g) > GREEN_MINIMUM_VALUE))

for img in os.listdir("green_false_positives/"):
	image = cv2.imread("green_false_positives/" + img)
	out = np.zeros((image.shape[0], image.shape[1], 1), dtype=np.uint8)

	for y in range(image.shape[0]):
		for x in range(image.shape[1]):
			if is_green(image[y, x, 0], image[y, x, 1], image[y, x, 2]):
				out[y, x] = 255

	cv2.imshow("In", image)
	cv2.imshow("Out", out)
	cv2.waitKey(0)