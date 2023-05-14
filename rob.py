import time
import math
import IMU
from smbus import SMBus

i2c = SMBus(1) #i2c start
addr = 0x30 #デバイスのアドレス 0x30
rot_time= 0.03 #5do rotate time

def rob_front_only(distance):
    
    go_time=distance*0.1
    
    i2c.write_byte(addr, ord('1'))
    print("@@前進中@@")
    time.sleep(go_time)
    print("進む秒数:",go_time)
    
    i2c.write_byte(addr, ord('0'))
    time.sleep(4)
    
    
    print("@@前進停止@@")
    
    
    

def rob_rotate(angle):
    
    roll,pitch,yaw =IMU.IMU_get() #IMU yaw get

    
    max_val=max(angle,yaw)
    min_val=min(angle,yaw)
    
    if max_val-min_val> 180:
        if (yaw<angle):
            rotate_val = 360-(max_val-min_val)
        else:
            rotate_val = -(360-(max_val-min_val))
    else:
        if(yaw<angle):
            rotate_val = max_val-min_val
        else:
            rotate_val = -(max_val-min_val)
    
    if(rotate_val>=0):
        i2c.write_byte(addr, ord('2'))
        print("@@右回転中@@")
        
        print('ロボットの向いている角度,目的地までの角度,回転すべき角度:')
        print(yaw,angle,rotate_val)
        
        time.sleep((rot_time/5)*rotate_val)
        i2c.write_byte(addr, ord('0'))
        print("@@右回転停止@@")
        
    if(rotate_val<0):
        i2c.write_byte(addr, ord('3'))
        print("@@左回転してます@@")
        
        print('ロボットの向いている角度,目的地までの角度,回転すべき角度:')
        print(yaw,angle,rotate_val)
        
        time.sleep((rot_time/5)*(-(rotate_val)))
        i2c.write_byte(addr, ord('0'))
        print("@@左回転停止@@")



