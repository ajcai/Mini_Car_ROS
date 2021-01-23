#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include </usr/include/opencv4/opencv2/highgui/highgui_c.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

static const std::string RGB_WINDOW = "RGB Image window";
//static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 0.6
#define Min_linear_speed 0.2
#define Min_distance 0.7
#define Max_distance 1.0
#define Max_rotation_speed 0.75

float linear_speed = 0;
float rotation_speed = 0;

float line_distance = 0;
float angle_distance = 0;
float angle_distance2 = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.004;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1 = 100;
int ERROR_OFFSET_X_left2 = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 540;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  std::string rgb_topic;
  std::string depth_topic;
  

  
public:
  ros::Publisher pub;

  ImageConverter()
    : it_(nh_)
  {

  nh_.param<std::string>("/kcf_tracker/rgb_topic", rgb_topic,"/camera/rgb/image_raw");
  nh_.param<std::string>("/kcf_tracker/depth_topic", depth_topic,"/camera/depth/image");
    // Subscrive to input video feed and publish output video feed
    /*image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image", 1, 
      &ImageConverter::depthCb, this);*/
    image_sub_ = it_.subscribe(rgb_topic, 1, 
      &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe(depth_topic, 1, 
      &ImageConverter::depthCb, this);
    pub = nh_.advertise<geometry_msgs::Twist>("kcf/track", 1000);
    //pub = nh_.advertise<geometry_msgs::Twist>("position/distance", 1000);
   

    cv::namedWindow(RGB_WINDOW);
    //cv::namedWindow(DEPTH_WINDOW); 
  }

  ~ImageConverter()
  {
    cv::destroyWindow(RGB_WINDOW);
    //cv::destroyWindow(DEPTH_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_ptr->image.copyTo(rgbimage);

    cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    if(bRenewROI)
    {
        // if (selectRect.width <= 0 || selectRect.height <= 0)
        // {
        //     bRenewROI = false;
        //     //continue;
        // }
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }

    if(bBeginKCF)
    {
        result = tracker.update(rgbimage);
        cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 1, 8 );
        enable_get_depth = true;
    }
    else
        cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

    cv::imshow(RGB_WINDOW, rgbimage);
    cv::waitKey(1);
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
  	cv_bridge::CvImagePtr cv_ptr;
  	try
  	{
  		cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  		cv_ptr->image.copyTo(depthimage);
  	}
  	catch (cv_bridge::Exception& e)
  	{
  		ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  	}

    if(enable_get_depth)
    {
      dist_val[0] = depthimage.at<float>(result.y+result.height/3 , result.x+result.width/3) ;
      dist_val[1] = depthimage.at<float>(result.y+result.height/3 , result.x+2*result.width/3) ;
      dist_val[2] = depthimage.at<float>(result.y+2*result.height/3 , result.x+result.width/3) ;
      dist_val[3] = depthimage.at<float>(result.y+2*result.height/3 , result.x+2*result.width/3) ;
      dist_val[4] = depthimage.at<float>(result.y+result.height/2 , result.x+result.width/2) ;

      float distance = 0;
      int num_depth_points = 5;
      for(int i = 0; i < 5; i++)
      {
        if(dist_val[i] > 0.4 && dist_val[i] < 10.0)
          distance += dist_val[i];
        else
          num_depth_points--;
      }
      distance /= num_depth_points;
      if(distance>0.6)  distance = distance;
        else            distance = 0.6;
      line_distance = distance;
 
      //calculate linear speed
      /*
      if(distance > Min_distance)
        linear_speed = distance * k_linear_speed + h_linear_speed;
      else
        linear_speed = 0;
      
      if(distance > Max_distance)
        linear_speed = 0.6;
      if(distance < Min_distance)
        linear_speed = -0.6;

      if(linear_speed > Max_linear_speed)
        linear_speed = Max_linear_speed;
*/
      //calculate rotation speed
      int center_x = result.x + result.width/2;
      int center_y = result.y + result.height/2;
      angle_distance2=center_y;
      angle_distance = center_x;
/*
      if(center_x < ERROR_OFFSET_X_left1) 
        rotation_speed =  Max_rotation_speed;
      else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
      else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
        rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
      else if(center_x > ERROR_OFFSET_X_right2)
        rotation_speed = -Max_rotation_speed;
      else 
        rotation_speed = 0;
*/
      std::cout <<  "linear_speed = " << linear_speed << "  rotation_speed = " << rotation_speed << std::endl;

      // std::cout <<  dist_val[0]  << " / " <<  dist_val[1] << " / " << dist_val[2] << " / " << dist_val[3] <<  " / " << dist_val[4] << std::endl;
      // std::cout <<  "distance = " << distance << std::endl;
    }

  	//cv::imshow(DEPTH_WINDOW, depthimage);
  	cv::waitKey(1);
  }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kcf_tracker");
	ImageConverter ic;
  /*
  private_nh.param<float>("Max_linear_speed", Max_linear_speed, 1.0); //固定串口
  private_nh.param<float>("Min_linear_speed", Min_linear_speed, 0.2); //和下位机底层波特率115200 不建议更高的波特率了
  private_nh.param<float>("Min_distance", Min_distance, 0.4);//平滑控制指令 
  private_nh.param<float>("Max_distance", Max_distance, 0.8);//ID
  private_nh.param<float>("Max_rotation_speed", Max_rotation_speed, 0.7);//ID
*/
	while(ros::ok())
	{
	  ros::spinOnce();

          geometry_msgs::Twist twist;

         twist.linear.x = line_distance; 
         twist.linear.y = 0; 
         twist.linear.z = 0;
         twist.angular.x = 0; 
         twist.angular.y = angle_distance2; 
         twist.angular.z = angle_distance;

         ic.pub.publish(twist);

    
         if (cvWaitKey(33) == 'q')
            break;
	}

	return 0;
}

