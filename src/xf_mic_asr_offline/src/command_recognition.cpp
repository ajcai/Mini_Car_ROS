/**************************************************************************
作者：caidx1
功能：命令控制器，命令词识别结果转化为对应的执行动作
**************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <iostream>
#include <stdio.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;
ros::Publisher vel_pub;    //创建底盘运动话题发布者
ros::Publisher cmd_vel_pub;
ros::Publisher follow_flag_pub;    //创建寻找声源标志位话题发布者
ros::Publisher cmd_vel_flag_pub;    //创建底盘运动控制器标志位话题发布者
ros::Publisher awake_flag_pub;    //创建唤醒标志位话题发布者
ros::Publisher navigation_auto_pub;    //创建自主导航目标点话题发布者
geometry_msgs::Twist cmd_msg;    //底盘运动话题消息数据
geometry_msgs::PoseStamped target;    //导航目标点消息数据

float I_position_x ;
float I_position_y ;
float I_orientation_z ;
float I_orientation_w ;

float J_position_x ;
float J_position_y ;
float J_orientation_z ;
float J_orientation_w ;

float K_position_x ;
float K_position_y ;
float K_orientation_z ;
float K_orientation_w ;

float line_vel_x ;
float ang_vel_z ;
float turn_line_vel_x ;
 
 
/**************************************************************************
函数功能：离线命令词识别结果sub回调函数
入口参数：命令词字符串  voice_control.cpp等
返回  值：无
**************************************************************************/
void voice_words_callback(const std_msgs::String& msg)
{
	/***指令***/
	string str1 = msg.data.c_str();    //取传入数据
	string str2 = "小车前进";
	string str3 = "小车后退"; 
	string str4 = "小车左转";
	string str5 = "小车右转";
	string str6 = "小车停";
	string str7 = "小车休眠";
	string str8 = "小车过来";
	string str9 = "小车去I点";
	string str10 = "小车去J点";
	string str11 = "小车去K点";
	string str12 = "失败5次";
	string str13 = "失败10次";
	string str14 = "遇到障碍物";
	string str15 = "小车唤醒";


/***********************************
指令：小车前进
动作：底盘运动控制器使能，发布速度指令
***********************************/
	if(str1 == str2)
	{
		cmd_msg.linear.x = line_vel_x;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
    cmd_vel_flag_msg.data = 1;
    cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车前进"<<endl;
	}
/***********************************
指令：小车后退
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str3)
	{
		cmd_msg.linear.x = -line_vel_x;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车后退"<<endl;
	}
/***********************************
指令：小车左转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str4)
	{
		cmd_msg.linear.x = turn_line_vel_x;
		cmd_msg.angular.z = ang_vel_z;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车左转"<<endl;
	}
/***********************************
指令：小车右转
动作：底盘运动控制器使能，发布速度指令
***********************************/
	else if(str1 == str5)
	{
		cmd_msg.linear.x = turn_line_vel_x;
		cmd_msg.angular.z = -ang_vel_z;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车右转"<<endl;
	}
/***********************************
指令：小车停
动作：底盘运动控制器失能，发布速度空指令
***********************************/
	else if(str1 == str6)
	{
		cmd_msg.linear.x = 0;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);


		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车停"<<endl;
	}
/***********************************
指令：小车休眠
动作：底盘运动控制器失能，发布速度空指令，唤醒标志位置零
***********************************/
	else if(str1 == str7)
	{
		cmd_msg.linear.x = 0;
		cmd_msg.angular.z = 0;
		vel_pub.publish(cmd_msg);

		std_msgs::Int8 awake_flag_msg;
        awake_flag_msg.data = 0;
        awake_flag_pub.publish(awake_flag_msg);

        std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 1;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"小车休眠，等待下一次唤醒"<<endl;
	}
/***********************************
指令：小车过来
动作：寻找声源标志位置位
***********************************/
	else if(str1 == str8)
	{
		std_msgs::Int8 follow_flag_msg;
		follow_flag_msg.data = 1;
		follow_flag_pub.publish(follow_flag_msg);
		cout<<"好的：小车寻找声源"<<endl;
	}
/***********************************
指令：小车去I点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str9)
	{
		target.pose.position.x = I_position_x;
		target.pose.position.y = I_position_y;
		target.pose.orientation.z = I_orientation_z;
		target.pose.orientation.w = I_orientation_w;
		navigation_auto_pub.publish(target);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 0;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车自主导航至I点"<<endl;
		
	}
/***********************************
指令：小车去I点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str10)
	{
		target.pose.position.x = J_position_x;
		target.pose.position.y = J_position_y;
		target.pose.orientation.z = J_orientation_z;
		target.pose.orientation.w = J_orientation_w;
		navigation_auto_pub.publish(target);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 0;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车自主导航至J点"<<endl;
	}
/***********************************
指令：小车去K点
动作：底盘运动控制器失能(导航控制)，发布目标点
***********************************/
	else if(str1 == str11)
	{
		target.pose.position.x = K_position_x;
		target.pose.position.y = K_position_y;
		target.pose.orientation.z = K_orientation_z;
		target.pose.orientation.w = K_orientation_w;
		navigation_auto_pub.publish(target);

		std_msgs::Int8 cmd_vel_flag_msg;
        cmd_vel_flag_msg.data = 0;
        cmd_vel_flag_pub.publish(cmd_vel_flag_msg);

		cout<<"好的：小车自主导航至K点"<<endl;
	}
/***********************************
辅助指令：失败5次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str12)
	{
		cout<<"您已经连续【输入空指令or识别失败】5次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：失败10次
动作：用户界面打印提醒
***********************************/
	else if(str1 == str13)
	{
		cout<<"您已经连续【输入空指令or识别失败】10次，累计达15次自动进入休眠，输入有效指令后计数清零"<<endl;
	}
/***********************************
辅助指令：遇到障碍物
动作：用户界面打印提醒
***********************************/
	else if(str1 == str14)
	{
		cout<<"小车遇到障碍物，已停止运动"<<endl;
	}
/***********************************
辅助指令：小车唤醒
动作：用户界面打印提醒
***********************************/
	else if(str1 == str15)
	{
		cout<<"小车已被唤醒，请说语音指令"<<endl;
	}
}

/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/
int main(int argc, char** argv)
{

	ros::init(argc, argv, "cmd_rec");     //初始化ROS节点

	ros::NodeHandle n;    //创建句柄
	
	string if_akm;
	
	/***创建寻找声源标志位话题发布者***/
	follow_flag_pub = n.advertise<std_msgs::Int8>("follow_flag",1);

	/***创建底盘运动控制器标志位话题发布者***/
	cmd_vel_flag_pub = n.advertise<std_msgs::Int8>("cmd_vel_flag",1);

	/***创建底盘运动话题发布者***/
	vel_pub = n.advertise<geometry_msgs::Twist>("ori_vel",10);

	cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	/***创建唤醒标志位话题发布者***/
	awake_flag_pub = n.advertise<std_msgs::Int8>("awake_flag", 1);

  /***创建自主导航目标点话题发布者***/
	navigation_auto_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);

	/***创建离线命令词识别结果话题订阅者***/
	ros::Subscriber voice_words_sub = n.subscribe("voice_words",10,voice_words_callback);


	n.param<float>("/I_position_x", I_position_x, 1);
	n.param<float>("/I_position_y", I_position_y, 0);
	n.param<float>("/I_orientation_z", I_orientation_z, 0);
	n.param<float>("/I_orientation_w", I_orientation_w, 1);
	n.param<float>("/J_position_x", J_position_x, 2);
	n.param<float>("/J_position_y", J_position_y, 0);
	n.param<float>("/J_orientation_z", J_orientation_z, 0);
	n.param<float>("/J_orientation_w", J_orientation_w, 1);
	n.param<float>("/K_position_x", K_position_x, 3);
	n.param<float>("/K_position_y", K_position_y, 0);
	n.param<float>("/K_orientation_z", K_orientation_z, 0);
	n.param<float>("/K_orientation_w", K_orientation_w, 1);
	n.param<float>("/line_vel_x", line_vel_x, 0.2);
	n.param<float>("/ang_vel_z", ang_vel_z, 0.2);
	n.param("/if_akm_yes_or_no", if_akm, std::string("no"));

	if(if_akm == "yes")
		turn_line_vel_x = 0.2;
	else 
		turn_line_vel_x = 0;

	/***自主导航目标点数据初始化***/
	target.header.seq = 0;
	//target.header.stamp;
	target.header.frame_id = "map";
	target.pose.position.x = 0;
	target.pose.position.y = 0;
	target.pose.position.z = 0;
	target.pose.orientation.x = 0;
	target.pose.orientation.y = 0;
	target.pose.orientation.z = 0;
	target.pose.orientation.w = 1;

  /***用户界面***/
	cout<<"您可以语音控制啦！唤醒词“你好小微”"<<endl;
	cout<<"小车前进———————————>向前"<<endl;
	cout<<"小车后退———————————>后退"<<endl;
	cout<<"小车左转———————————>左转"<<endl;
	cout<<"小车右转———————————>右转"<<endl;
	cout<<"小车停———————————>停止"<<endl;
	cout<<"小车休眠———————————>休眠，等待下一次唤醒"<<endl;
	cout<<"小车过来———————————>寻找声源"<<endl;
	cout<<"小车去I点———————————>小车自主导航至I点"<<endl;
	cout<<"小车去J点———————————>小车自主导航至J点"<<endl;
	cout<<"小车去K点———————————>小车自主导航至K点"<<endl;
	cout<<"\n"<<endl;
	//printf("-----turn_line_vel_x =%f\n",turn_line_vel_x);
	//printf("-----I_position_x =%f\n",I_position_x);
	//printf("-----I_position_y =%f\n",I_position_y);
	//printf("-----I_orientation_z =%f\n",I_orientation_z);
	//printf("-----I_orientation_w =%f\n",I_orientation_w);
	//printf("-----J_position_x =%f\n",J_position_x);
	//printf("-----J_position_y =%f\n",J_position_y);
	//printf("-----J_orientation_z =%f\n",J_orientation_z);
	//printf("-----J_orientation_w =%f\n",J_orientation_w);
	//printf("-----K_position_x =%f\n",K_position_x);
	//printf("-----K_position_y =%f\n",K_position_y);
	//printf("-----K_orientation_z =%f\n",K_orientation_z);
	//printf("-----K_orientation_w =%f\n",K_orientation_w);
	//printf("-----line_vel_x =%f\n",line_vel_x);
	//printf("-----ang_vel_z =%f\n",ang_vel_z);
	ros::spin();
}

