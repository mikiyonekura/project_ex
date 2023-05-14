import net #net.py imort
import GPS_get #GPS get import
import IMU #IMU import
import theta_distance#回転角　距離算出
import rob5
import time
from collections import defaultdict
import os
from pathlib import Path
import networkx as nx
import cv2
import math
import RPi.GPIO as GPIO

#jet:BOARD-31 to Pi:BCM-6

GPIO.setmode(GPIO.BCM) #ラズパイはBCMモードでGPIOを読み込む

GPIO.setup(6,GPIO.IN) #QR結果用　0:読み取れてない　1:読み取れた

QR_GPIO_VAL=0

distance=5


while(QR_GPIO_VAL == 0):
    rob5.rob_front_only(distance)
    QR_GPIO_VAL=GPIO.input(6)

print('QR yomikonda !!')


