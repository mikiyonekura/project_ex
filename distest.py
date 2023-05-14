# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.OUT)#2^0
GPIO.setup(27,GPIO.OUT)#2^1
GPIO.setup(22,GPIO.OUT)#2^2
GPIO.setup(5,GPIO.OUT)#2^3 予定
#QR結果用　0:読み取れてない　1:読み取れた
GPIO.setup(6,GPIO.IN)

GPIO.output(17,True)
GPIO.output(27,True)
GPIO.output(22,False)
GPIO.output(5,False)


