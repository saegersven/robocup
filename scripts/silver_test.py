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
	# IF VALUE BETWEEN 100 AND 210:
	# CHECK IF PIXEL IS RED (RATIO > 0.6)

	MIN_VALUE = 100
	MIN_C_VALUE = 160
	MIN_RATIO = 0.58

	COORD_C = (28, 30)
	SIZE_C = (10, 20)

	COORD_L = (28, 21)
	SIZE_L = (10, 11)

	COORD_R = (28, 50)
	SIZE_R = (10, 11)

	roi_L = img[COORD_L[0]:COORD_L[0]+SIZE_L[0], COORD_L[1]:COORD_L[1]+SIZE_L[1]]
	roi_R = img[COORD_R[0]:COORD_R[0]+SIZE_R[0], COORD_R[1]:COORD_R[1]+SIZE_R[1]]
	roi_C = img[COORD_C[0]:COORD_C[0]+SIZE_C[0], COORD_C[1]:COORD_C[1]+SIZE_C[1]]

	average_L = roi_L.mean(axis=0).mean(axis=0)
	average_R = roi_R.mean(axis=0).mean(axis=0)
	v_C = roi_C.mean(axis=0).mean(axis=0).mean(axis=0)

	r_L = average_L[2] / (average_L[1] + average_L[0])
	v_L = average_L.mean(axis=0)

	r_R = average_R[2] / (average_R[1] + average_L[0])
	v_R = average_R.mean(axis=0)

	return r_L > MIN_RATIO and r_R > MIN_RATIO and v_C > MIN_C_VALUE

num_silver = 0

images = glob.glob("ml_data/silver/robot_detected_silver/*.png")

for i, filename in enumerate(images):
	img = cv2.imread(filename)

	print(f"{filename}:\t{silver_current(img)}")