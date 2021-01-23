/**************************************************************************
作者：caidx1
功能：
**************************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <simple_follower/position.h>


using namespace std;

nav_msgs::Odometry odom;

sensor_msgs::Imu Imu_Data;

float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向
int a;
//float radar_range ;
//int radar_count ;

/**************************************************************************
函数功能：sub回调函数
入口参数：  laserTracker.py
返回  值：无
**************************************************************************/
void current_position_Callback(const simple_follower::position& msg)	
{
	distance1 = msg.distance;
	dis_angleX = msg.angleX;

}


/**************************************************************************
函数功能：底盘运动sub回调函数（原始数据）
入口参数：cmd_msg  command_recognition.cpp
返回  值：无
**************************************************************************/
void vel_Callback(const nav_msgs::Odometry& msg)
{
	odom.twist.twist.linear.x = msg.twist.twist.linear.x;
	odom.twist.twist.linear.y = msg.twist.twist.linear.y;
	
}


void acc_Callback(const sensor_msgs::Imu& msg)
{
	Imu_Data.linear_acceleration.x = msg.linear_acceleration.x;
	Imu_Data.linear_acceleration.y = msg.linear_acceleration.y;
	Imu_Data.angular_velocity.z = msg.angular_velocity.z;
}



/**************************************************************************
函数功能：判断障碍物距离是否小于0.75米
入口参数：无
返回  值：1或0
**************************************************************************/
int distance_judgment(void)
{
	//int a;
	if(distance1<=0.75) 
		return 1;
	else
		return 0;
	
}
 
/**************************************************************************
函数功能：判断障碍物方向是否在小车运动趋势方向上
入口参数：无
返回  值：1或0
**************************************************************************/



int vel_judgment(void)
{
	if(odom.twist.twist.linear.x > 0 && (dis_angleX > 2.335 || dis_angleX < -2.33))
		return 1;

	else if(odom.twist.twist.linear.x < 0 && dis_angleX > -0.785 && dis_angleX < 0.785)
		return 1;
		
	else if(odom.twist.twist.linear.y > 0 && dis_angleX < 0)
		return 1;

	else if(odom.twist.twist.linear.y < 0 && dis_angleX > 0)
		return 1;
		
	else 
		return 0;
}

int acc_judgment(void)
{
	if(Imu_Data.linear_acceleration.x > 0.02 && (dis_angleX > 2.335 || dis_angleX < -2.335))
	{
		a=1;
		return 1;
	}

	else if(Imu_Data.linear_acceleration.x < -0.02 && dis_angleX > -0.785 && dis_angleX < 0.785)
	{
		a=1;
		return 1;
	}
	//else if(Imu_Data.linear_acceleration.y > 0.01 && dis_angleX < 0)
	//	return 1;

	//else if(Imu_Data.linear_acceleration.y < -0.01 && dis_angleX > 0)
	//	return 1;

	else 
		return 0;

}

int dis_angleX_judgment(void)
{
	if(vel_judgment())//||acc_judgment())
		return 1;

	else 
		return 0;					
}



/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{
	int turn_fin_flag=0;    //完成转向标志位
	int temp_count = 0;    //计数变量


	ros::init(argc, argv, "obs_avo");    //初始化ROS节点

	ros::NodeHandle node;    //创建句柄

	/***创建底盘速度控制话题发布者***/
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe("/object_tracker/current_position", 1, current_position_Callback);

	ros::Subscriber vel_sub = node.subscribe("/odom", 1, vel_Callback);

	ros::Subscriber acc_sub = node.subscribe("/mobile_base/sensors/imu_data", 1, acc_Callback);


	
	double rate2 = 20;    //频率10Hz
	ros::Rate loopRate2(rate2);

	//node.param("/radar_range", radar_range);
	//node.param("/radar_count", radar_count); 


	while(ros::ok())
	{		
		if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向
		{
			
			temp_count++;
			if(temp_count > 0)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点
			{
				int i=0;
				if(i<50)
				{
					cmd_vel_Pub.publish(geometry_msgs::Twist());
					i++;
					
				}
				temp_count = 0;
				a=0;
			}
		}
		else temp_count = 0;    //排除雷达噪点
		
		ros::spinOnce();
		loopRate2.sleep();
	} 



	return 0;
}

