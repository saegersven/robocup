import RPi.GPIO as GPIO
import time
pin = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, GPIO.HIGH)
time.sleep(5)
GPIO.output(pin, GPIO.LOW)