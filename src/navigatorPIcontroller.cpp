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
#include <math.h>
#include "../include/thruster_control/navigatorPIcontroller.h"

#define SAT				5
#define NUM_SENSORS		2

//DEBUG Flags
#define DEBUG_FLAG_BOOL	1
#define DEBUG_FLAG_DATA	0
#define DEBUG_FLAG_BACK	0

using namespace std;


NavPiController::NavPiController() : as_ (nh, "GoToPoseAction", false)

{
	enableExecution		= false;
	targetPosition		= false;
	userControlRequest	= false;
	
	for (int i=0; i<=NUM_SENSORS; i++)
		safetyMeasureAlarm.data.push_back(0);
	
	//Publishers initialization
	pub_odom = nh.advertise<nav_msgs::Odometry>("dataNavigator", 1);
	
	//Subscribers initialization
	sub_odomInfo = nh.subscribe<geometry_msgs::Pose>("g500/pose", 1, &NavPiController::odomCallback, this);
	sub_safetyInfo = nh.subscribe<std_msgs::Int8MultiArray>("safetyMeasures", 1, &NavPiController::safetyMeasuresCallback,this);

	//Action initilization
	as_.registerGoalCallback(boost::bind(&NavPiController::executeCB, this));
	as_.registerPreemptCallback(boost::bind(&NavPiController::preemptCB, this));
	as_.start();
	ROS_INFO("Action server initialized");
}


NavPiController::~NavPiController()
{
	//Destructor
}


/************************************************************************/
/*						Action Server section							*/
/************************************************************************/
void NavPiController::executeCB()
{
	thruster_control::goToPoseGoalConstPtr goal_;
	targetPosition = false;
	initMissionTime = ros::Time::now();	
	feedback_.action="Initializing GoToPose().action";
	as_.publishFeedback(feedback_);

	goal_ = as_.acceptNewGoal();
	enableExecution = goal_->startAction;

	stationKeeping = goal_->stationKeeping;
	robotDesiredPosition.pose.position.x = goal_->robotTargetPosition.position.x;
	robotDesiredPosition.pose.position.y = goal_->robotTargetPosition.position.y;
	robotDesiredPosition.pose.position.z = goal_->robotTargetPosition.position.z;
	
	if (!stationKeeping)
		distError = 0.2;
	else
		distError = 0;

	feedback_.action="Starting GoToPose().action";
	as_.publishFeedback(feedback_);
	
	goToPose();

	feedback_.action="GoToPose().action finished";
	as_.publishFeedback(feedback_);
	
	if ((!targetPosition) and (!enableExecution))
	{
		result_.succeed = false;	//Mission finished with error
		feedback_.action="Mission finished with error";
	}
	if ((targetPosition) and (enableExecution))
	{
		result_.succeed = true;		//Mission finished successfully
		feedback_.action="Mission finished successfully";
	}
	as_.publishFeedback(feedback_);
	as_.setSucceeded(result_);
}

void NavPiController::preemptCB()
{
	ROS_INFO("Goal Preempted");
	feedback_.action="Goal Preempted";
	as_.setPreempted();
	as_.publishFeedback(feedback_);
	result_.succeed = false;
	as_.setAborted(result_);
}

/************************************************************************/
/*							FUNCTIONS									*/
/************************************************************************/
void NavPiController::odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue)
{
	double currentErrorDist;
	
	//Updating the current robot position & orientation
	robotCurrentPose.position    = odomValue->position;
	robotCurrentPose.orientation = odomValue->orientation;

	//Storing the last robot-target position
	robotLastPose.position = robotTargetPose.position;

	//Getting the difference between target position & current pose.
	robotTargetPose.position.x = robotDesiredPosition.pose.position.x - robotCurrentPose.position.x;
	robotTargetPose.position.y = robotDesiredPosition.pose.position.y - robotCurrentPose.position.y;
	robotTargetPose.position.z = robotDesiredPosition.pose.position.z - robotCurrentPose.position.z;

	//Getting the difference between last & current pose.
	robotErrorPose.position.x = robotTargetPose.position.x - robotLastPose.position.x;
	robotErrorPose.position.y = robotTargetPose.position.y - robotLastPose.position.y;
	robotErrorPose.position.z = robotTargetPose.position.z - robotLastPose.position.z;

	lastRobotTargetDist = currentRobotTargetDist;

	//Checking if the robot has achieved the target position
	currentRobotTargetDist = sqrt( (double)(pow(robotTargetPose.position.x, 2)) \
		+ (double)(pow(robotTargetPose.position.y, 2)) + (double)(pow(robotTargetPose.position.z, 2)) );
	if ((currentRobotTargetDist < distError) and (enableExecution))
	{
		targetPosition = true;
	}
	
	//Checking if the robot is stopped
	currentErrorDist = lastRobotTargetDist - currentRobotTargetDist;
	currentMissionTime = ros::Time::now();
	totalMissionTime = currentMissionTime - initMissionTime;
	
	//To Fix: we need to control when the robot is stopped
/*	cout << "currentRobotTargetDist = " << currentRobotTargetDist << endl;
	cout << "currentErrorDist = " << currentErrorDist << endl;
	cout << "totalMissionTime = " << totalMissionTime.toSec() << endl;
	if ((abs(currentErrorDist) < 0.000001) and (enableExecution) and (totalMissionTime.toSec() > 2.0) )
	{
		//enableExecution = false;				//Error aquí!!!!!
		cout << "robot is stopped??" << endl;
	}*/

	if (DEBUG_FLAG_BOOL)
	{
		cout << "currentRobotTargetDist = " << currentRobotTargetDist << endl;

		if (enableExecution)
			cout << "The robot is working. enableExecution = " << enableExecution << endl;
		else
			cout << "The robot has a problem. enableExecution = " << enableExecution << endl;

		if (targetPosition)
			cout << "The robot has achieved the target position. targetPosition = " << targetPosition << endl;
		else
			cout << "The robot has not achieved the target position. targetPosition = " << targetPosition << endl;

		if (((int) safetyMeasureAlarm.data[0]) != 0)
			cout << "Safety alarm!!! The user has the robot control." << endl;

		if (((int) safetyMeasureAlarm.data[NUM_SENSORS+1]) != 0)
			cout << "The user has requested the robot control." << endl;
	}
	if (DEBUG_FLAG_DATA)
	{
		cout << "currentErrorDist = " << currentErrorDist << endl;
		cout << "totalMissionTime = " << totalMissionTime.toSec() << endl;
		cout << "robotErrorPose  = " << robotErrorPose.position.x << ", " << \
				robotErrorPose.position.y << ", " << robotErrorPose.position.z << endl;
		cout << "robotTargetPose = " << robotTargetPose.position.x << ", " << \
				robotTargetPose.position.y << ", " << robotTargetPose.position.z << endl; 
	}
	ros::spinOnce();
}


void NavPiController::goToPose()
{
	double Itermx = 0; double Itermy = 0; double Itermz = 0;
	double errorx = 0; double errory = 0; double errorz = 0;
	double gain = 0.5, Igain = 0;	

	ros::Rate loop_rate(50);
	
	while ((enableExecution) and (!targetPosition))
	{
		if ((!userControlRequest) )
		{
			//Compute control law
			errorx = gain * (robotTargetPose.position.x);
			errory = gain * (robotTargetPose.position.y);
			errorz = gain * (robotTargetPose.position.z);

			Itermx += robotTargetPose.position.x;
			Itermy += robotTargetPose.position.y;
			Itermz += robotTargetPose.position.z;

			//Send message to Simulator
			nav_msgs::Odometry msg;
			msg.twist.twist.linear.x  = errory + Igain * Itermy;
			msg.twist.twist.linear.y  = -errorx - Igain * Itermx;
			msg.twist.twist.linear.z  = errorz + Igain * Itermz;
			msg.twist.twist.angular.x = 0;
			msg.twist.twist.angular.y = 0;
			msg.twist.twist.angular.z = 0;
			if (msg.twist.twist.linear.x > SAT) msg.twist.twist.linear.x = SAT; else if (msg.twist.twist.linear.x < -SAT) msg.twist.twist.linear.x = -SAT;
			if (msg.twist.twist.linear.y > SAT) msg.twist.twist.linear.y = SAT; else if (msg.twist.twist.linear.y < -SAT) msg.twist.twist.linear.y = -SAT;
			if (msg.twist.twist.linear.z > SAT) msg.twist.twist.linear.z = SAT; else if (msg.twist.twist.linear.z < -SAT) msg.twist.twist.linear.z = -SAT;
			pub_odom.publish(msg);

			if (DEBUG_FLAG_DATA)
			{
				cout << "msg.twist.twist" << endl;
				cout << msg.twist.twist << endl;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//Send 0 velocity to the thrustersAllocator rospackage
	nav_msgs::Odometry msg;
	msg.twist.twist.linear.x  = 0;
	msg.twist.twist.linear.y  = 0;
	msg.twist.twist.linear.z  = 0;
	msg.twist.twist.angular.x = 0;
	msg.twist.twist.angular.y = 0;
	msg.twist.twist.angular.z = 0;
	pub_odom.publish(msg);
}


/************************************************************************/
/*							CALLBACKS									*/
/************************************************************************/
void NavPiController::safetyMeasuresCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
	for (int i=0; i<=NUM_SENSORS+1; i++)
		safetyMeasureAlarm.data[i] = msg->data[i];
		
	userControlRequest = msg->data[NUM_SENSORS+1];

	if (DEBUG_FLAG_BACK)
	{
		cout << "safetyMeasureAlarm: [";
		for (int i=0; i<=NUM_SENSORS+1; i++)
			cout << safetyMeasureAlarm.data[i] << " ";
		cout << "]" << endl;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigatorPIcontroller");
	NavPiController navPiControl;
	ros::spin();
}
