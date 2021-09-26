from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import os
import os.path

folder = str(input("Input folder name: "))
res = (160, 96)

camera = PiCamera()
camera.resolution = res

num_images = len([name for name in os.listdir(folder) if os.path.isfile(os.path.join(folder, name))]) + 1
GPIO.setmode(GPIO.BOARD)

GPIO.setup(35, GPIO.IN)

try:
	while True:
		input("Press enter to capture image: ")
		pic_name = str(num_images) + ".jpg"
		camera.capture(os.path.join(folder, pic_name))
		print(f"Captured '{pic_name}'")
		num_images = num_images + 1
except KeyboardInterrupt:
	camera.close()