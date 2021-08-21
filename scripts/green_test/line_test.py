import cv2
import numpy as np

image = cv2.imread("test_images/1.jpg")

green = cv2.inRange(image, (0, 60, 0), (40, 255, 40))
white = cv2.inRange(image, (150, 150, 150), (255, 255, 255))

contours_grn, hierarchy = cv2.findContours(green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

contours, hierarchy = cv2.findContours(white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

rect = cv2.minAreaRect(np.concatenate(contours_grn))

box = cv2.boxPoints(rect) # cv2.boxPoints(rect) for OpenCV 3.x
box = np.int0(box)
cv2.drawContours(image, [box], 0, (0, 0, 255), 2)

for i in range(len(contours)):
	cv2.drawContours(image, [contours[i]], 0, (0, 255, 0), 2)

cv2.imshow("Image", image)
cv2.waitKey(5000)
cv2.destroyAllWindows()