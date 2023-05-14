# GPS_get.py

# -*- coding: utf-8 -*-
import serial
import micropyGPS
import threading
import time
import simplekml

gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット

def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    while True:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        if sentence[0] != '$': # 先頭が'$'でなければ捨てる
            continue
        for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)

def rungps_kai(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
    if sentence[0] == '$': # 先頭が'$'でなければ捨てる
        for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
#gpsthread = threading.Thread(target=rungps, args=()) # 上の関数を実行するスレッドを生成
#gpsthread.daemon = True
#gpsthread.start() # スレッドを起動
def gps_get():
    rungps_kai()
    x=0.0
    y=0.0
    for i in range(10):
        #print(type(gps.latitude[0]))
        
        x=x+gps.latitude[0]#ido 10times
        y=y+gps.longitude[0]#keido 10times
    x=x/10 #ido avg
    y=y/10 #keido avg

    #x= gps.latitude[0]
    #y= gps.longitude[0]
    return x,y;#return ido keido
"""
while True:
    if gps.clean_sentences > 20: # ちゃんとしたデーターがある程度たまったら出力する
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
        print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
        print('海抜: %f' % gps.altitude)
        print(gps.satellites_used)
        print('衛星番号: (仰角, 方位角, SN比)')
        for k, v in gps.satellite_data.items():
            print('%d: %s' % (k, v))
        print('')
    time.sleep(3.0)


#if __name__ == "__main__":
#    print("$
"""