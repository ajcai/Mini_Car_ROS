#!/usr/bin/env python 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python3安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python3的安装路径，再调用对应路径下的解析器完成操作
#2:Python3.X 源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

import cv2 #引用opencv功能包
import numpy as np #引用数组功能包

#调阈值回调函数
def callback(object):
    pass
        
#创建窗口    
cv2.namedWindow('MyWindow')

#提示停止方法
print ('Showing camera. Press key "Q" to stop.')

cameraCapture = cv2.VideoCapture(0) #创建读取摄像头的类
succes, frame = cameraCapture.read() #读取第一帧图片，返回值为(是否成功读取， 图片)
frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
#创建画布、窗口、进度条
canvas = np.zeros((170, 600, 3), dtype=np.uint8) + 255 #创建画布放置阈值动态调节窗口
cv2.imshow("THRESHOLD",canvas)
cv2.createTrackbar("R_min","THRESHOLD",0,255,callback) #输入参数(参数名字，进度条附着窗口名字，进度条最小值，进度条最大值，回调函数)
cv2.createTrackbar("R_max","THRESHOLD",0,255,callback)  
cv2.createTrackbar("G_min","THRESHOLD",0,255,callback)
cv2.createTrackbar("G_max","THRESHOLD",0,255,callback)
cv2.createTrackbar("B_min","THRESHOLD",0,255,callback)
cv2.createTrackbar("B_max","THRESHOLD",0,255,callback)
cv2.createTrackbar("kernel_width","THRESHOLD",1,20,callback)
cv2.createTrackbar("kernel_height","THRESHOLD",1,20,callback)

Quit=0 #是否继续运行标志位
while succes  and not Quit:
    keycode=cv2.waitKey(1)
    if(keycode==ord('Q')): #如果按下“Q”键，停止运行标志位置1，调出while循环，程序停止运行
       Quit=1

    #进度条值赋值相关变量
    R_min = cv2.getTrackbarPos("R_min","THRESHOLD",) #获得进度条值
    G_min = cv2.getTrackbarPos("G_min","THRESHOLD",)
    B_min = cv2.getTrackbarPos("B_min","THRESHOLD",)
    R_max = cv2.getTrackbarPos("R_max","THRESHOLD",)
    G_max = cv2.getTrackbarPos("G_max","THRESHOLD",)
    B_max = cv2.getTrackbarPos("B_max","THRESHOLD",)
    kernel_width=cv2.getTrackbarPos("kernel_width","THRESHOLD",)
    kernel_height=cv2.getTrackbarPos("kernel_height","THRESHOLD",)
    if(kernel_width<1):kernel_width=1
    if(kernel_height<1):kernel_height=1

    #分别对RGB三通道进行二值化
    b, g ,r =cv2.split(frame)      #RGB通道分离
    #cv2.imshow('r', r)#通道R
    #cv2.imshow('g', g)#通道G
    #cv2.imshow('b', b)#通道B
    frame_threshold_B = cv2.inRange(b,B_min,B_max) #通道R二值化
    frame_threshold_G = cv2.inRange(g,G_min,G_max) #通道G二值化
    frame_threshold_R = cv2.inRange(r,R_min,R_max) #通道B二值化
    #cv2.imshow('frame_threshold_R', frame_threshold_R)#窗口显示二值化后的通道R
    #cv2.imshow('frame_threshold_G', frame_threshold_G)#窗口显示二值化后的通道G
    #cv2.imshow('frame_threshold_B', frame_threshold_B)#窗口显示二值化后的通道B
    
    #3通道二值化结果相与
    Binary_RGB_AND = cv2.bitwise_and(frame_threshold_R, frame_threshold_G) #相与
    frame_binary_manual = cv2.bitwise_and(Binary_RGB_AND, frame_threshold_B)#最终结果
    #cv2.imshow('frame_binary_manual', frame_binary_manual) #窗口显示彩色图像手动二值化结果
    
    #直接对RGB图像进行二值化
    lower_rgb = np.array([B_min, G_min, R_min]) #注意这里是BGR,
    upper_rgb = np.array([B_max, G_max, R_max]) #因为opencv读取图片的默认像素排列是BGR
    frame_binary = cv2.inRange(frame,lower_rgb,upper_rgb) #使用数组进行图像二值化

    #创建膨胀腐蚀核并进行膨胀腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))
    frame_binary_DE = cv2.erode(frame_binary, kernel)
    frame_binary_DE = cv2.dilate(frame_binary_DE, kernel)
    frame_binary_DE = cv2.dilate(frame_binary_DE, kernel)
    frame_binary_DE = cv2.erode(frame_binary_DE, kernel)
    
    #cv2.imshow('MyWindow', frame) #窗口显示原图
    #cv2.imshow('frame_binary', frame_binary)#窗口显示对RGB图像进行二值化的结果
    cv2.imshow('frame_binary_DE', frame_binary_DE)#窗口显示膨胀腐蚀结果
    
    succes, frame = cameraCapture.read() #循环读取摄像头
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
if succes==0: #提示由于摄像头读取失败停止程序
    print ('Camera disconnect !') 
print ('Quitted!') #提示程序已停止
cv2.destroyAllWindows() #程序停止前关闭所有窗口
cameraCapture.release #程序停止前关闭摄像头调用


