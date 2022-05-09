import cv2
import numpy as np
import os

IMAGE_DIR = "../../ml_data/victims/victims"

for img in os.listdir(IMAGE_DIR):
	image = cv2.imread(IMAGE_DIR + "/" + img, cv2.IMREAD_GRAYSCALE)
	blurred = cv2.GaussianBlur(image, (9, 9), 0, 0)

	circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1.5,
		50,
		param1=65, param2=55, minRadius=0, maxRadius=0
	)

	if(circles is not None):
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
		    # draw the outer circle
		    cv2.circle(blurred, (i[0], i[1]), i[2], (0, 255, 0), 2)
		    # draw the center of the circle
		    cv2.circle(blurred, (i[0], i[1]), 2, (0, 0, 255), 3)

	cv2.imshow('detected circles', blurred)
	cv2.waitKey(1000)
cv2.destroyAllWindows()