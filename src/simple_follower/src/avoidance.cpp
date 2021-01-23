/**************************************************************************
作者：caidx1
功能：雷达避障
**************************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <simple_follower/position.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>




using namespace std;
 


geometry_msgs::Twist cmd_vel_msg;    //速度控制信息数据

float distance1;    //障碍物距离
float dis_angleX;    //障碍物方向

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
void cmd_vel_ori_Callback(const geometry_msgs::Twist& msg)
{
	cmd_vel_msg.linear.x = msg.linear.x;
	cmd_vel_msg.angular.z = msg.angular.z;
	
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
int dis_angleX_judgment(void)
{
	if(cmd_vel_msg.linear.x > 0 && (dis_angleX > 1.57 || dis_angleX < -1.57))
		return 1;

	else if(cmd_vel_msg.linear.x < 0 && dis_angleX > -1.57 && dis_angleX < 1.57)
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
	int temp_count = 0;    //计数变量
	string str1 = "遇到障碍物";    //障碍物字符串

	ros::init(argc, argv, "avoidance");    //初始化ROS节点

	ros::NodeHandle node;    //创建句柄

	/***创建底盘速度控制话题发布者***/
	ros::Publisher cmd_vel_Pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/***创建底盘运动话题订阅者***/
	ros::Subscriber vel_sub = node.subscribe("cmd_vel_ori", 1, cmd_vel_ori_Callback);

  	/***创建障碍物方位话题订阅者***/
	ros::Subscriber current_position_sub = node.subscribe("/object_tracker/current_position", 1, current_position_Callback);

	
	double rate2 = 10;    //频率10Hz
	ros::Rate loopRate2(rate2);

 
	while(ros::ok())
	{
		ros::spinOnce();
			
		if(distance_judgment() && dis_angleX_judgment())    //判断障碍物的距离和方向
		{
			temp_count++;
			if(temp_count > 5)    //连续计数5️次停止运动防止碰撞，避免雷达有噪点
			{

				cmd_vel_Pub.publish(geometry_msgs::Twist());
				temp_count = 0;
			}
		}
		else
		{
			temp_count = 0;    //排除雷达噪点
			cmd_vel_Pub.publish(cmd_vel_msg);    //将速度指令发送给机器人
		}

		ros::spinOnce();
		loopRate2.sleep();
	} 


	return 0;
}
