/* 
 * Copyright (c) 2015 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Author:
 * 		Javier Pérez Soler
 *      Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include <hrov_control/HrovControlStdMsg.h>

using namespace std;

class NavPiController
{

	public:
		NavPiController();
		~NavPiController();
		
		bool 			enableExecution;
		bool			targetPosition;
		bool			userControlRequest;
		double			lastRobotTargetDist;
		double			currentRobotTargetDist;

		ros::Time		initMissionTime;
		ros::Time		currentMissionTime;
		ros::Duration	totalMissionTime;

		std_msgs::Int8MultiArray	safetyAlarm;

		
		void GoToPose();
		

		
	private:
		ros::NodeHandle nh;

		ros::Publisher		pub_odom;
		ros::Subscriber		sub_odomInfo;
		ros::Subscriber		sub_safetyInfo;
		ros::Subscriber		sub_userControlInfo;
		ros::ServiceServer	runBlackboxGotoPoseSrv;
		
		geometry_msgs::Pose			robotCurrentPose;
		geometry_msgs::Pose			robotLastPose;
		geometry_msgs::Pose			robotTargetPose;		//Dif: targetPose - CurrentPose
		geometry_msgs::Pose			robotErrorPose;			//Dif: currentPose - LastPose
		geometry_msgs::PoseStamped  robotDesiredPosition;	//Where the robot should go

		void odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue);
		void safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
		void userControlReqCallback(const std_msgs::Bool::ConstPtr& msg);
		bool enableRunBool(hrov_control::HrovControlStdMsg::Request &req, hrov_control::HrovControlStdMsg::Response &res);

};
