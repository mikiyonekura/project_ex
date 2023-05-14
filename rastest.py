import  RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(17,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)
GPIO.setup(22,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)

GPIO.setup(6,GPIO.IN)

while True:
    GPIO.output(17,False)
    GPIO.output(27,True)
    GPIO.output(22,False)
    GPIO.output(5,True)
    
    input_val6 = GPIO.input(6)
    print(input_val6)