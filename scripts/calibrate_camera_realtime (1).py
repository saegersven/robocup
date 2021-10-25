# small script to calibrate the camera/get the right color values for black, green, red and blue

# TODO:
# avoid saving image on harddrive
# Convert BGR results to HSV
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import statistics
from time import sleep

number_measurements = int(input("How often to you want to calibrate each color?: "))

def get_frame():	
	camera = PiCamera()
	camera.resolution = (640, 384)
	camera.capture("image.jpg")	
	camera.close()
	sleep(0.05)
	return cv2.imread("image.jpg")


def color_of_pixel(img, x, y):
	return img[y, x]

def take_measurement(img):
	res_blue = []
	res_green = []
	res_red = []

	for x in range(302, 339): # loop through roi and save all pixel colors
		for y in range(174, 211):
			color = color_of_pixel(img, x, y)
			res_blue.append(color[0])
			res_green.append(color[1])
			res_red.append(color[2])

	return (statistics.mean(res_blue), statistics.mean(res_green), statistics.mean(res_red)) # return tuple of color averages

def preview(color, cnt):
	camera = PiCamera()
	camera.resolution = (640, 384)
	camera.rotation = 0
	camera.framerate = 70
	rawCapture = PiRGBArray(camera, size=(640, 384))

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		cv2.rectangle(image, (300, 172), (340, 212), (0, 255, 255), 2)
		cv2.putText(image, "Calibrating: " + color, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 2)
		cv2.putText(image, "Measurement : " + str(cnt), (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 106, 255), 2)

		cv2.imshow("Image", image)
		key = cv2.waitKey(1) & 0xFF
		if key == ord("w"):			
			camera.close()
			return
		rawCapture.truncate(0)

black_vals = []
blue_vals = []
green_vals = []
red_vals = []


for i in range(number_measurements):
	preview("Black", i + 1)
	sleep(1)
	black_vals.append(take_measurement(get_frame()))

for i in range(number_measurements):
	preview("Blue", i + 1)
	sleep(1)
	blue_vals.append(take_measurement(get_frame()))

for i in range(number_measurements):
	preview("Green", i + 1)
	sleep(1)
	green_vals.append(take_measurement(get_frame()))

for i in range(number_measurements):
	preview("Red", i + 1)
	sleep(1)
	red_vals.append(take_measurement(get_frame()))

print("\n\n\n\n########### RESULTS (RAW) ###########")

print(f"BLACK (BGR): {black_vals}")
print(f"BLUE (BGR): {blue_vals}")
print(f"GREEN (BGR): {green_vals}")
print(f"RED(BGR): {red_vals}")


print("\n\n\n\n########### RESULTS (AVG) ###########")

print(f"BLACK (BGR): {tuple(map(np.mean, zip(*black_vals)))}")
print(f"BLUE (BGR): {tuple(map(np.mean, zip(*blue_vals)))}")
print(f"GREEN (BGR): {tuple(map(np.mean, zip(*green_vals)))}")
print(f"RED (BGR): {tuple(map(np.mean, zip(*red_vals)))}")


	