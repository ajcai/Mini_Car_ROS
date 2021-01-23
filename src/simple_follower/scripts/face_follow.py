#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy
from geometry_msgs.msg import Twist
last_erro=0
face_cascade = cv2.CascadeClassifier( '/home/wheeltec/wheeltec_robot/src/ros_simple_follower/simple_follower/scripts/face.xml' ) 
eye_cascade = cv2.CascadeClassifier('/home/wheeltec/wheeltec_robot/src/ros_simple_follower/simple_follower/scripts/eye.xml')

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        # 订阅usb摄像头
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("cv_bridge_image", Image, self.image_callback)
        # 订阅深度相机
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/depth/image_raw", Image,self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()


    def image_callback(self, msg):
        global last_erro
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale( gray )
        frame = image
        if len(faces)>0:
            for (x,y,w,h) in faces:
                #参数分别表示“目标图片”，“矩形左上角坐标”，“矩形右下角坐标”，“线条颜色”，“线宽”
                face = cv2.rectangle(image,(x,y),(x+h,y+w),(255,0,0),2)
                i,m,g = face.shape
                roi_gray = gray[y: y + h, x: x + w]
                roi_color = image[y: y + h, x: x + w]
                eyes = eye_cascade.detectMultiScale(roi_gray)
                for(ex, ey, ew, eh) in eyes:
                    cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 0, 255), 2)
                    eye = cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 0, 255), 2)
            img = cv2.rectangle(image,(x,y),(x+h,y+w),(255,0,0),2)
            img[y:y+w, x:x+h] = [0,0,255]
            #img[x:x+h, 0:b] = [0,0,255]
            #img[x:a, 0:b] = [0,0,0]
            frame = img
            # cv2.imshow("window4", img)
            # hsv将RGB图像分解成色调H，饱和度S，明度V
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            kernel = numpy.ones((5,5),numpy.uint8)
            hsv_erode = cv2.erode(hsv,kernel,iterations=1)
            hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
            lower_black = numpy.array([0, 43, 46])
            upper_black = numpy.array([9, 255, 255])
            mask = cv2.inRange(hsv_dilate, lower_black, upper_black)
            masked = cv2.bitwise_and(frame, frame, mask=mask)
            a, b, c = image.shape
            M = cv2.moments(mask)
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(frame, (cx, cy), 10, (255, 0, 0), -1)
                if cv2.circle:
            # 计算图像中心线和目标指示线中心的距离
                    erro = cx - b/2
                    d_erro=erro-last_erro
                    #self.twist.linear.x = 0.3
                    if erro<0:
                        self.twist.angular.z = -float(erro)*0.001-float(d_erro)*0.01 
                    else:
                        self.twist.angular.z = -float(erro)*0.001-float(d_erro)*0.01   
                        #self.twist.angular.z = 0
                        #self.twist.angular.z = erro
                    last_erro=erro
                    if self.twist.angular.z>1:
                        self.twist.angular.z = 1
                    elif self.twist.angular.z<-1:
                        self.twist.angular.z = -1
                    self.cmd_vel_pub.publish(self.twist)
                    rospy.logwarn(erro)
            frame = img
        #cv2.imshow("window", image)
        #cv2.imshow("window1", hsv)
        #cv2.imshow("window2", hsv_erode)
        #cv2.imshow("window", frame)
            #cv2.imshow("window4", mask)
        cv2.waitKey(3)
rospy.init_node("opencv")
follower = Follower()
rospy.spin()

