import cv2
import glob
import matplotlib.pyplot as plt

def clip(val, lower, higher):
	if val < lower:
		return lower
	elif val > higher:
		return higher
	else:
		return val

def silver_current(img):
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (9, 9), 0)

	white = cv2.inRange(blurred, (10), (90))

	cv2.imshow("White", white)
	cv2.waitKey(1000)


num_silver = 0

images = glob.glob("../ml_data/silver/silver_images_sorted/*.png")

for i, filename in enumerate(images):
	img = cv2.imread(filename)

	print(f"{filename}:\t{silver_current(img)}")