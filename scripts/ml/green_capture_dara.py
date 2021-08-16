from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import os, os.path

def capture(path, res):
	camera = PiCamera()
	camera.resolution = res
	camera.capture(path)

folder = str(input("Input folder name: "))
res = (80, 48)

num_images = len([name for name in os.listdir(folder) if os.path.isfile(os.path.join(folder, name))]) + 1

GPIO.setmode(GPIO.BOARD)

GPIO.setup(35, GPIO.IN)

while True:
	if GPIO.input(35) == GPIO.HIGH:
		pic_name = str(num_images) + ".jpg"
		capture(os.path.join(folder, pic_name), res)
		print(f"Captured '{pic_name}'")
		num_images = num_images + 1
camera.close()