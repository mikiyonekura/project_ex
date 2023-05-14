# -*- coding: utf-8 -*-
from smbus import SMBus
import time
import math
import datetime
import csv

# I2C
ACCL_ADDR = 0x19
ACCL_R_ADDR = 0x02
GYRO_ADDR = 0x69
GYRO_R_ADDR = 0x02
MAG_ADDR = 0x13
MAG_R_ADDR = 0x42

i2c = SMBus(1)


# ・・・
#  センサデータ取得関数
# ・・・
def bmx_setup():
    # acc_data_setup : 加速度の値をセットアップ
    i2c.write_byte_data(ACCL_ADDR, 0x0F, 0x03)
    i2c.write_byte_data(ACCL_ADDR, 0x10, 0x08)
    i2c.write_byte_data(ACCL_ADDR, 0x11, 0x00)
    time.sleep(0.5)

    # gyr_data_setup : ジャイロ値をセットアップ
    i2c.write_byte_data(GYRO_ADDR, 0x0F, 0x04)
    i2c.write_byte_data(GYRO_ADDR, 0x10, 0x07)
    i2c.write_byte_data(GYRO_ADDR, 0x11, 0x00)
    time.sleep(0.5)

    # mag_data_setup : 地磁気値をセットアップ
    data = i2c.read_byte_data(MAG_ADDR, 0x4B)
    if(data == 0):
        i2c.write_byte_data(MAG_ADDR, 0x4B, 0x83)
        time.sleep(0.5)
    i2c.write_byte_data(MAG_ADDR, 0x4B, 0x01)
    i2c.write_byte_data(MAG_ADDR, 0x4C, 0x00)
    i2c.write_byte_data(MAG_ADDR, 0x4E, 0x84)
    i2c.write_byte_data(MAG_ADDR, 0x51, 0x04)
    i2c.write_byte_data(MAG_ADDR, 0x52, 0x16)
    time.sleep(0.5)
    
def acc_value():
    data = [0, 0, 0, 0, 0, 0]
    acc_data = [0.0, 0.0, 0.0]

    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(ACCL_ADDR, ACCL_R_ADDR + i)

        for i in range(3):
            acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
            if acc_data[i] > 2047:
                acc_data[i] -= 4096
            acc_data[i] *= 0.0098

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return acc_data

def gyro_value():
    data = [0, 0, 0, 0, 0, 0]
    gyro_data = [0.0, 0.0, 0.0]

    try:
        for i in range(6):
            data[i] = i2c.read_byte_data(GYRO_ADDR, GYRO_R_ADDR + i)

        for i in range(3):
            gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
            if gyro_data[i] > 32767:
                gyro_data[i] -= 65536
            gyro_data[i] *= 0.0038

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return gyro_data

def mag_value():
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    mag_data = [0.0, 0.0, 0.0]

    try:
        for i in range(8):
            data[i] = i2c.read_byte_data(MAG_ADDR, MAG_R_ADDR + i)

        for i in range(3):
            if i != 2:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                if mag_data[i] > 4095:
                    mag_data[i] -= 8192
            else:
                mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                if mag_data[i] > 16383:
                    mag_data[i] -= 32768

    except IOError as e:
        print("I/O error({0}): {1}".format(e.errno, e.strerror))

    return mag_data

def initial_error(acc):    # オイラー角の初期値誤差を算出
    roll = math.degrees(math.atan2(acc[1] , math.sqrt(acc[0]**2+acc[2]**2)))
    pitch = -math.degrees(math.atan2(acc[0] , math.sqrt(acc[1]**2+acc[2]**2)))

    return -roll, -pitch

#for test
def IMU_get():
    #bmx_setup()
    #time.sleep(0.1)
    acc = acc_value()    #センサの初期値を取得
    roll_error, pitch_error = initial_error(acc)    # 初期値を誤差へ
    
    roll=[]
    pitch=[]
    yaw=[]
    
    for i in range(10):
        acc = acc_value()
        gyro = gyro_value()
        mag = mag_value()

        roll.append(math.atan2(acc[1] , math.sqrt(acc[0]**2+acc[2]**2)))
        pitch.append(-math.atan2(acc[0] , math.sqrt(acc[1]**2+acc[2]**2)))
        #print("OK")
        numerator = math.cos(roll[i])*mag[1] - math.sin(roll[i])*mag[2]
        denominator = math.cos(pitch[i])*mag[0] + math.sin(pitch[i])*math.sin(roll[i])*mag[1] + math.sin(pitch[i])*math.cos(roll[i])*mag[2]
        yaw.append(math.atan2(numerator,denominator))

        roll[i] = math.degrees(roll[i]) + roll_error
        pitch[i] = math.degrees(pitch[i]) + pitch_error
        yaw[i] = math.degrees(yaw[i])
        
        yaw[i]=yaw[i]-7.16#偏角を考慮　明野付近は７度10分
        if yaw[i] < 0:
            yaw[i]=yaw[i]+360
    #yaw avg
    yaw_avg=sum(yaw)/len(yaw)
    #print("roll:{}".format(round(roll,2)))
    #print("pitch:{}".format(round(pitch,2)))
    print("yaw:{}".format(round(yaw_avg,2)))
    print("\n")
    return roll,pitch,yaw_avg
