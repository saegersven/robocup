import RPi.GPIO as GPIO
import time
pin = 20
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, GPIO.HIGH)
time.sleep(5)
GPIO.output(pin, GPIO.LOW)