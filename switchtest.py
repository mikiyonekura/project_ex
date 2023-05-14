import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BCM)

GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

while True:
    print(GPIO.input(26))
    time.sleep(1)