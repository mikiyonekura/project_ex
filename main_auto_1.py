# main_auto_1.py

# -*- coding: utf-8 -*-
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
#import cv2
import math
import RPi.GPIO as GPIO
from smbus import SMBus
import threading

i2c = SMBus(1) #i2c start
addr = 0x30 #デバイスのアドレス 0x30

"""RasPi - Jetson 通信用定義甲(始)"""
#jet:BOARD-19 to Pi:BCM-17
#jet:BOARD-21 to Pi:BCM-27
#jet:BOARD-23 to Pi:BCM-22
#jet:BOARD-29 to Pi:BCM-5
#jet:BOARD-31 to Pi:BCM-6
#BOARDはJETSONの基盤番号,BCMはGPIO番号

#ラズパイはBCMモードでGPIOを読み込む
GPIO.setmode(GPIO.BCM)
#マップ出力用GPIO
GPIO.setup(17,GPIO.OUT)#2^0
GPIO.setup(27,GPIO.OUT)#2^1
GPIO.setup(22,GPIO.OUT)#2^2
GPIO.setup(5,GPIO.OUT)#2^3 予定
#QR結果用　0:読み取れてない　1:読み取れた
GPIO.setup(6,GPIO.IN)
#ノード到達スイッチ用
GPIO.setup(26,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

#ノード・エッジ定義
#longtitude : latitude
#d[###][0] : lattiude
#d[###][1] : longtitude
#コース3は狭いので距離に30足した重み，曲がり角ではとりあえず距離＋10の重み
#コース2と3はGPSが取りづらいので+20
#コース1 > コース2 > コース3
#体育館前 > S/Cの間　> 一番上のルート
NODE_LIST = ["1000", "2000", "3000", "4000", "5000","6000","7000","8000","9000","10000","11000","12000","13000","14000"]
EDGE_LIST = [
    ("1000", "2000", 24.019),
    ("1000", "8000", 4.6+50),
    ("2000", "3000", 20.18),
    ("2000", "13000", 3.4+10+20),
    ("3000", "4000", 7.58+10),
    ("4000", "5000", 20.78+10),
    ("5000", "6000", 77.02+10),
    ("6000", "7000", 17.82+10),
    ("8000", "9000", 7.78+20),
    ("9000", "10000", 76.536+20),
    ("10000", "11000", 25.51+10+20),
    ("11000", "12000", 42.058),
    ("12000", "7000", 17.75+10),
    ("13000", "14000", 9.64+20),
    ("14000", "11000", 75.852+20),
]
#スタートノード
start = "1000"
#ゴールノード
end = "7000"

#ノード・エッジ定義を基に無向グラフ作成
DG = nx.Graph()
DG.add_nodes_from(NODE_LIST)
DG.add_weighted_edges_from(EDGE_LIST)
#ノード1000から7000までの最適経路算出
shortest_path = nx.astar_path(DG, start, end)
#ディスプレイ出力用(コース1)
dis_flag = 1
print("dis_flag:",dis_flag)
print("Shortest Path:", shortest_path)

#最適経路からノード1000を消す
del shortest_path[0]       
short = shortest_path

#ノードの緯度経度を定義
d = defaultdict(list)
#node : 1000
d["1000"].append(33.232437131)#[0]
d["1000"].append(131.650675465)#[1]
#node : 2000
d["2000"].append(33.232221475)#[0]
d["2000"].append(131.650683683)#[1]
#node : 3000
d["3000"].append(33.232040203)#[0]
d["3000"].append(131.650683835)#[1]
#node : 4000
d["4000"].append(33.232035173)#[0]
d["4000"].append(131.650764987)#[1]
#node : 5000
d["5000"].append(33.231848702)#[0]
d["5000"].append(131.650775882)#[1]
#node : 6000
d["6000"].append(33.231853555)#[0]
d["6000"].append(131.651603016)#[1]
#node : 7000
d["7000"].append(33.231693518)#[0]
d["7000"].append(131.651608997)#[1]
#node : 8000
d["8000"].append(33.232449518)#[0]
d["8000"].append(131.650722773)#[1]
#node : 9000
d["9000"].append(33.232453545)#[0]
d["9000"].append(131.650806235)#[1]
#node : 10000
d["10000"].append(33.232457647)#[0]
d["10000"].append(131.651628182)#[1]
#node : 11000
d["11000"].append(33.232228605)#[0]
d["11000"].append(131.651637768)#[1]
#node : 12000
d["12000"].append(33.231850799)#[0]
d["12000"].append(131.651640046)#[1]
#node : 13000
d["13000"].append(33.232220526)#[0]
d["13000"].append(131.650719888)#[1]
#node : 14000
d["14000"].append(33.232215955)#[0]
d["14000"].append(131.650823288)#[1]

def route_search(bef_node,aft_node):
    global dis_flag
    DG.remove_edge(bef_node,aft_node)
    start = bef_node
    end = "7000"
    shortest_path = nx.astar_path(DG, start, end)
    #shortest_path_weight = nx.astar_path_length(DG, start, end)
    if(shortest_path[0] == "1000"):
        if(shortest_path[1] == "2000"):
            dis_flag = 1 #original.png
        else :
            dis_flag = 2 #1000-2000.png
    elif(shortest_path[0] == "2000"):
        if(shortest_path[1] == "13000"):
            dis_flag = 3#2000-3000.png
        else:
            dis_flag = 7#2000to1000.png
    elif(shortest_path[0] == "3000"):
        dis_flag = 4#3000-4000.png
    elif(shortest_path[0] == "4000"):
        dis_flag = 5#4000-5000.png
    elif(shortest_path[0] == "5000"):
        dis_flag = 6#5000-6000.png
    elif(shortest_path[0] == "13000"):
        dis_flag = 8#13000.png
    elif(shortest_path[0] == "14000"):
        dis_flag = 9#14000.png
    elif(shortest_path[0] == "11000"):
        dis_flag = 10#11000.png
    else:
        dis_flag = 11#例外

    return shortest_path,dis_flag

def map_display(dis_flag):
    #(w,x,y,z)
    if(dis_flag == 1):
        w = False
        x = False 
        y = False
        z = True
        return w,x,y,z
    elif(dis_flag == 2):
        w = False
        x = False
        y = True
        z = False
        return w,x,y,z
    elif(dis_flag == 3):
        w = False
        x = False
        y = True
        z = True
        return w,x,y,z
    elif(dis_flag == 4):
        w = False
        x = True
        y = False
        z = False
        return w,x,y,z
    elif(dis_flag == 5):
        w = False
        x = True
        y = False
        z = True
        return w,x,y,z
    elif(dis_flag == 6):
        w = False
        x = True
        y = True
        z = False
        return w,x,y,z
    elif(dis_flag == 7):
        w = False
        x = True
        y = True
        z = True
        return w,x,y,z
    elif(dis_flag == 8):
        w = True
        x = False
        y = False
        z = False
        return w,x,y,z
    elif(dis_flag == 9):
        w = True
        x = False
        y = False
        z = True
        return w,x,y,z
    elif(dis_flag == 10):
        w = True
        x = False
        y = True
        z = False
        return w,x,y,z
    else:#1111はエラー
        w = True
        x = True
        y = True
        z = True
        return w,x,y,z


###ここから移動開始###
notEndnode = True

flag_for = 1 #flag_forは1と2を交互に回して最経路探索に対応
bef_node = "1000" #初期値ノード

IMU.bmx_setup()

#avoidance state get (thread)

# def arduino_state():
#     global ard_state
#     #ard_state = i2c.read_byte(addr)
#     ard_state = 3
#     print("@@@ arduino state @@@",ard_state)
# 
# thread1 = threading.Thread(target= arduino_state)
# thread1.start()
#ゴールにいない限り永遠に回るループ
while notEndnode:
    if(flag_for==1):
        for i in short:
            
            #dis_flagをもとに二進数へ変換
            #(disp1,disp2,disp3,disp4)
            disp1,disp2,disp3,disp4 = map_display(dis_flag)
            GPIO.output(17,disp4)
            GPIO.output(27,disp3)
            GPIO.output(22,disp2)
            GPIO.output(5,disp1)
            aft_node = i
            notNextnode = True

            #次のノードへたどりついていない限りルー
            while notNextnode:
                ard_state = i2c.read_byte(addr)
                print("for文１における状態は：",ard_state)
                if(ard_state == 3):
                    short,dis_flag = route_search(bef_node,aft_node)
                    flag_for=2
                    print("現在のフラグは:",flag_for)
                    break
                                         
                x,y=GPS_get.gps_get()#x=ido y= keido
                print('緯度と軽度は:',x,y)
                #距離と角度算出関数
                #(現在地の緯度,現在地の経度,目的地の緯度,目的地の経度)
                distance,angle1,angle2=theta_distance.vincenty_inverse(x, y, d[i][0], d[i][1], 1)
                
                print("====現在の状況を表示====")
                print("現在のフラグは:",flag_for)
                print("今向かっているノード：",i)
                print("目的地までの距離:",distance)
                print("====出力終了====")
                qr_switch = GPIO.input(26)
                if(distance<=3 or qr_switch == 1):
                    QR_GPIO_VAL=GPIO.input(6)
                    print(QR_GPIO_VAL)
                    while (QR_GPIO_VAL == 0): #QR_GPIO_VAL is Gloval val
                        print('QR読み込み中')
                        rob5.rob_front_only(2.5)
                        QR_GPIO_VAL=GPIO.input(6)
                        print(QR_GPIO_VAL)
                    print(QR_GPIO_VAL)    
                    print('QR読みこみ完了!!')
                    
                    nigasanai = True
                    while nigasanai:
                        
                        print("switchオフにしようね")
                        time.sleep(1)
                        qr_switch = GPIO.input(26)
                        if(qr_switch == 0):
                            nigasanai = False
                    
                    
                        
                    notNextnode = False
                    print("********目的のノードに到着********")

                if(distance<2000):
                    rob5.rob_rotate(angle1)
                    rob5.rob_front_only(distance)
                
                time.sleep(2)
        
            print("for文2における状態は：",ard_state)
            if(ard_state == 3):
                print("flag_for:2へ移行します")
                break
            
            if(i == "7000"):
                notEndnode = False
                
            bef_node = aft_node
            
    if(flag_for==2):
        for i in short:
            
            #dis_flagをもとに二進数へ変換
            #(disp1,disp2,disp3,disp4)
            disp1,disp2,disp3,disp4 = map_display(dis_flag)
            GPIO.output(17,disp4)
            GPIO.output(27,disp3)
            GPIO.output(22,disp2)
            GPIO.output(5,disp1)
            aft_node = i
            notNextnode = True

            #次のノードへたどりついていない限りループ
            while notNextnode:
                ard_state = i2c.read_byte(addr)
                print("for文2における状態は：",ard_state)
                if(ard_state == 3):
                    short,dis_flag = route_search(bef_node,aft_node)
                    flag_for=1
                    print("--リルートしたあとの経路--",short)
                    break       
                    
                x,y=GPS_get.gps_get()#x=ido y= keido
                print('緯度と軽度は:',x,y)
                #距離と角度算出関数
                #(現在地の緯度,現在地の経度,目的地の緯度,目的地の経度)
                distance,angle1,angle2=theta_distance.vincenty_inverse(x, y, d[i][0], d[i][1], 1)
                
                print("====現在の状況を表示====")
                print("現在のフラグは:",flag_for)
                print("今向かっているノード：",i)
                print("目的地までの距離:",distance)
                print("====出力終了====")
                
                qr_switch = GPIO.input(26)
                if(distance<=3 or qr_switch == 1): 
                    QR_GPIO_VAL=GPIO.input(6)
                    print("QR読み込みの値は:",QR_GPIO_VAL)
                    while (QR_GPIO_VAL == 0): #QR_GPIO_VAL is Gloval val
                        print('QR読み込み中')
                        rob5.rob_front_only(2)
                        QR_GPIO_VAL=GPIO.input(6)
                        print(QR_GPIO_VAL)
                    print(QR_GPIO_VAL)    
                    print('QR読み込んだ!!')
                    
                    nigasanai = True
                    while nigasanai:
                        
                        print("switchオフにしようね")
                        time.sleep(1)
                        qr_switch = GPIO.input(26)
                        if(qr_switch == 0):
                            nigasanai = False
                    
                    
                    notNextnode = False
                    print("********次のノードに到着********")

                if(distance<2000):
                    rob5.rob_rotate(angle1)
                    rob5.rob_front_only(distance)
                
                time.sleep(2)
            print("for文2における状態は：",ard_state)
            if(ard_state == 3):
                print("flag_for1へ移行します")
                break
            
            if(i == "7000"):
                notEndnode = False
                
            bef_node = aft_node

print("ーーーー処理終了ーーーー")