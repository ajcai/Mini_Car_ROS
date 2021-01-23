#!/usr/bin/env python 
# coding=utf-8
#1.编译器声明和2.编码格式声明
#1:为了防止用户没有将python3安装在默认的/usr/bin目录，系统会先从env(系统环境变量)里查找python3的安装路径，再调用对应路径下的解析器完成操作
#2:Python3.X 源码文件默认使用utf-8编码，可以正常解析中文，一般而言，都会声明为utf-8编码

import rospy #引用ROS的Python接口功能包
import cv2, cv_bridge #引用opencv功能包。cv_bridge是ROS图像消息和OpenCV图像之间转换的功能包
from sensor_msgs.msg import Image #引用ROS内的图片消息格式
import message_filters

#定义一个图片转换的类，功能为：订阅ROS图片消息并转换为OpenCV格式处理，处理完成再转换回ROS图片消息后发布
class Image_converter:
 def __init__(self): #类成员初始化函数   
     self.bridge = cv_bridge.CvBridge() #初始化图片转换功能，cv_bridge.CvBridge()
     #message_filters的作用是把订阅的数据同步后再在message_filters的回调函数中使用
     im_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
     dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
     self.resize_rgb_pub = rospy.Publisher("resize_image_rgb", Image, queue_size=1)
     self.resize_depth_pub = rospy.Publisher("resize_image_depth", Image, queue_size=1)
     self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
     self.timeSynchronizer.registerCallback(self.VisualFollow)
 
 def VisualFollow(self, image_data, depth_data):
      #图像转换与预处理            
      frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')            #ROS图像转OpenCV图像
      depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#ROS图像转OpenCV图像
      frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)            #降低图像分辨率，以提高程序运行速度        
      depthFrame = cv2.resize(depthFrame, (320,240), interpolation=cv2.INTER_AREA)  #降低图像分辨率，以提高程序运行速度    
      self.resize_rgb_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8")) 
      self.resize_depth_pub.publish(self.bridge.cv2_to_imgmsg(depthFrame, "mono16"))    
      cv2.waitKey(1)

if __name__ == '__main__': #这段判断的作用是，如果本py文件是直接运行的则判断通过执行if内的内容，如果是import到其他的py文件中被调用(模块重用)则判断不通过
  rospy.init_node("ImageResize") #创建节点
  rospy.loginfo("ImageResize node started") #打印ROS消息说明节点已开始运行
  Image_converter() #直接运行image_converter()函数创建类，该类在运行期间会一直存在。因为该类没有需要调用的函数，所以使用赋值的形式：a=image_converter()
  rospy.spin() #相当于while(1),当订阅者接收到新消息时调用回调函数

