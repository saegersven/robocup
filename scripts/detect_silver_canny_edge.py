import cv2
import random

image = cv2.imread("./images/silver_test_images/2.jpg")

resized = cv2.resize(image, (400, 240))

while True:
	# highly effictive stuff going on here:
	p1 = random.randint(0, 300)
	p2 = random.randint(0, 300)

	edges = cv2.Canny(image, p1, p2)
	edges = cv2.resize(edges, (400, 240)) # display image a bit bigger

	cv2.imshow("Original", resized)
	cv2.imshow("Canny Edge", edges)

	print(f"p1: {p1}, p2: {p2}")
	cv2.waitKey(0)