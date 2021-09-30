print("Help!")
import RPi.GPIO as GPIO
import time
pin = 38
GPIO.setmode(GPIO.BCM)
print("Help2!")

GPIO.setup(pin, GPIO.OUT)
print("Help3!")

GPIO.output(pin, GPIO.HIGH)
print("Help4!")
time.sleep(1)
GPIO.output(pin, GPIO.LOW)