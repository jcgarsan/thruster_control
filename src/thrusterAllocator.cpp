//#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8MultiArray.h>
#include "ros/ros.h"
#include "underwater_sensor_msgs/DVL.h"

#define SAT	5
#define DEBUG_thrusterAllocator	1
#define NUM_SENSORS				2


//Twist callback to get velocity reference
class TwistCallback
{
  public:
    double linear[3];
    double angular[3];

    TwistCallback()
    {
		linear[0]=linear[1]=linear[2]=0;
		angular[0]=angular[1]=angular[2]=0;
    }
 
    void callback(const nav_msgs::Odometry& msg)
    {
      linear[0]=msg.twist.twist.linear.x;
      linear[1]=msg.twist.twist.linear.y;
      linear[2]=msg.twist.twist.linear.z;

      angular[0]=msg.twist.twist.angular.x;
      angular[1]=msg.twist.twist.angular.y;
      angular[2]=msg.twist.twist.angular.z;

      //std::cout<<linear[0]<<" "<<linear[1]<<" "<<linear[2]<<std::endl;
    }
};

//DVL callback to get vehicle's velocity
class DVLCallback
{
  public:
    double linear[3];

    DVLCallback()
    {
		linear[0]=linear[1]=linear[2]=0;
    }
 
    void callback(const underwater_sensor_msgs::DVL& msg)
    {
		linear[0]=msg.bi_x_axis;
		linear[1]=msg.bi_y_axis;
		linear[2]=msg.bi_z_axis;

		//std::cout<<linear[0]<<" "<<linear[1]<<" "<<linear[2]<<std::endl;
    }
};


//SafetyAlarmCallback avoids publishing thruster data when safetyAlarm is true
class SafetyAlarmCallback
{
	public:
		std_msgs::Int8MultiArray	safetyAlarmData;

		SafetyAlarmCallback()
		{
			for (int i=0; i<=NUM_SENSORS; i++)
				safetyAlarmData.data.push_back(0);
		}

		void callback(const std_msgs::Int8MultiArray::ConstPtr& msg)
		{
			for (int i=0; i<=NUM_SENSORS+1; i++)
				safetyAlarmData.data[i] = msg->data[i];
		}
};


int main(int argc, char **argv)
{
  std::string twist_topic, thrusters_topic, dvl_topic, safetyAlarm_topic; 
  TwistCallback twist;
  DVLCallback dvl;
  SafetyAlarmCallback safetyAlarm;

  ros::init(argc, argv, "vehicleThrusterAllocator");
  ros::NodeHandle nh;
 
  nh.param("twist", twist_topic, (std::string)"/dataNavigator");
  nh.param("thrusters", thrusters_topic, (std::string)"/g500/thrusters_input");
  nh.param("dvl", dvl_topic, (std::string)"/g500/dvl");
  nh.param("safetyAlam", safetyAlarm_topic, (std::string)"/safetyMeasures");

  ros::Subscriber sub_twist = nh.subscribe(twist_topic, 1000, &TwistCallback::callback,&twist);
  ros::Subscriber sub_dvl = nh.subscribe(dvl_topic, 1000, &DVLCallback::callback,&dvl);
  ros::Subscriber sub_safetyAlarm = nh.subscribe(safetyAlarm_topic, 1000, &SafetyAlarmCallback::callback,&safetyAlarm);
  ros::Publisher  pub=nh.advertise<std_msgs::Float64MultiArray>(thrusters_topic, 1);

  ros::Rate loop_rate(200);

  double thrust_req[5];
  double last_thrust_req[5];
  double current_thrust_req[5];
  memset(last_thrust_req, 0, sizeof(last_thrust_req)); //clear array

  sleep(10);
  while(ros::ok()) {
    memset(thrust_req, 0, sizeof(thrust_req)); //clear array
    
	thrust_req[0]=(-twist.linear[0]-dvl.linear[0])*5 - twist.angular[2];
	thrust_req[1]=(-twist.linear[0]-dvl.linear[0])*5 + twist.angular[2];
	thrust_req[0]=(-twist.linear[0]-dvl.linear[0])*5;
	thrust_req[1]=(-twist.linear[0]-dvl.linear[0])*5;
	thrust_req[2]=-(twist.linear[2]-dvl.linear[2])*5;
	thrust_req[3]=-(twist.linear[2]-dvl.linear[2])*5;
	thrust_req[4]=(twist.linear[1]+dvl.linear[1])*6;

    //Send message to UWSim when user doesn't request the robot control
    //or there is any safetyAlarm
	if ((int) safetyAlarm.safetyAlarmData.data[NUM_SENSORS+1] == 0)	
	{
		std_msgs::Float64MultiArray msg;
		for (int i=0; i<5; i++)
			msg.data.push_back(thrust_req[i]);
		pub.publish(msg);
	}
	
	if (DEBUG_thrusterAllocator)
	{
		if ((int) safetyAlarm.safetyAlarmData.data[NUM_SENSORS+1] == 0)	
			std::cout << "Thrusters array: " << thrust_req[0] << ", " << thrust_req[1] << ", " << thrust_req[2] << \
					", " <<thrust_req[3] << ", " << thrust_req[4] << std::endl;
		else
			std::cout << "The user has the robot control" << std::endl;
	}

    ros::spinOnce();
    loop_rate.sleep();
  }
}
