////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  real UAV obstacle avoidance aplication (simple version fro debbuging v.0)									    
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

// Most common ros headers
#include <ros/ros.h>

// Action headers
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_px4/changeHeadingAction.h>
#include <my_px4/hoverHeightAction.h>
#include <boost/thread.hpp>

// Messages headers
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

// General headers
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <vector>

// Distance definitions
#define MANEUVER_VELOCITY	0.5f //velocity during the maneuver
#define ALIGN_DISTANCE		1.5f //allow to move this distance away from
								 //the last obstacle
#define MAX_CLEAR_DIST  	2.0f //require at least that distance clear to attempt
								 //a maneuver in this direction
#define MAX_HOVER_HEIGHT    4.0f //this is the limit of the sensor which report
								 //the distance from the ground
#define MAX_MANEUVER_HEIGHT 3.5f //do not allow vertical maneuver over that
#define HOVER_HEIGHT		1.0f //the altitude that you set for the hover
#define DEADEND_DISTANCE    5.0f //this is the limit where you decide that you
								 //in deadend (distance in the left and right)
#define MAX_LASER_DIST     40.0f //the max range of the LIDARLite is 40m
#define MIN_LASER_DIST  	0.3f //this is the base of the UAV which reaported as obstacle
								 //from the laser
#define MIN_FRONT_DIST 		2.0f //under that limit an object is sensed in front
#define EMERGENCY_DIST		1.0f //do not allow movement if sence object closer
								 //to this

using namespace std;

class ObstacleAvoidance3 {
	public:
		ObstacleAvoidance3();
		
		void taxi(char mode); //taxi mode
		void lift(float height); //lift to a height
		void freeze(); //freeze
		
		//state of the FSM
		int state;
		ros::Time lastSwitch;
		//distances around the UAV
		double frontDistance;
		double leftDistance, rightDistance;
		double currentHeight, currentHeading;
	
	private:
		ros::NodeHandle n; //create a handle for the ROS node
		//list of subscribers
		ros::Subscriber scan0_sub, scan90_sub, scanm90_sub, cmdvel_sub, sheight_sub, imu_sub; 
		//list of publishers
		ros::Publisher cmdvel_pub, hover_pub;// heading_pub;
		
		//callback functions of the subscribers
		void laser0CallBack(const std_msgs::Float32::ConstPtr& scan0_msg);
		void laser90CallBack(const std_msgs::Float32::ConstPtr& scan90_msg);
		void laserm90CallBack(const std_msgs::Float32::ConstPtr& scanm90_msg);
		void cmdvelGetawayCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmdvel_msg);
		void sonarHeightCallBack(const std_msgs::Float32::ConstPtr& heightMsg);
		void imuCallBack(const sensor_msgs::Imu::ConstPtr& heightMsg);
		
		//helper methods
		bool emergencyCheck();
	};
	
	ObstacleAvoidance3::ObstacleAvoidance3() {
		
		//subscribe to the related topics
		scan0_sub = n.subscribe<std_msgs::Float32>("/dist0", 100, &ObstacleAvoidance3::laser0CallBack, this);
		scan90_sub = n.subscribe<std_msgs::Float32>("/dist90", 100, &ObstacleAvoidance3::laser90CallBack, this);
		scanm90_sub = n.subscribe<std_msgs::Float32>("/distm90", 100, &ObstacleAvoidance3::laserm90CallBack, this);
		
		cmdvel_sub = n.subscribe<geometry_msgs::TwistStamped>("/obs_cmd_vel", 100, &ObstacleAvoidance3::cmdvelGetawayCallBack, this);
		sheight_sub = n.subscribe<std_msgs::Float32>("/sonar", 100, &ObstacleAvoidance3::sonarHeightCallBack, this);
		imu_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 1, &ObstacleAvoidance3::imuCallBack, this);
		
		//advertise the publisher's topics in the ROS using the master
		cmdvel_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 100);
		hover_pub = n.advertise<geometry_msgs::Point>("/hover_height", 100);
	}
	
	void ObstacleAvoidance3::cmdvelGetawayCallBack(const geometry_msgs::TwistStamped::ConstPtr& cmdvel_msg) {
		//if there is no obstacle keep transmitting what received from the imputs (keyboard or joystic)
		//to the normal topic for movement 'cmd_vel'. This is the manual mode - no intervation
		//If for any reason sense something (emergencyChack) closer than 1m go to freeze
		geometry_msgs::TwistStamped cmdvel_command_msg;
	
		if (state == 0) {
			ros::Time my_time = ros::Time::now();
			
			cmdvel_command_msg.header.stamp = cmdvel_msg->header.stamp;
			cmdvel_command_msg.twist.linear.x = cmdvel_msg->twist.linear.x;
			cmdvel_command_msg.twist.linear.y = cmdvel_msg->twist.linear.y;
			cmdvel_command_msg.twist.linear.z = cmdvel_msg->twist.linear.z;
			cmdvel_command_msg.twist.angular.x = cmdvel_msg->twist.angular.x;
			cmdvel_command_msg.twist.angular.y = cmdvel_msg->twist.angular.y;
			cmdvel_command_msg.twist.angular.z = cmdvel_msg->twist.angular.z;
			
			cmdvel_pub.publish(cmdvel_command_msg);
			
			ROS_INFO("Manual mode...\n");
			
			if (true == emergencyCheck()) {
				state = 3;
			}
		} 
	}
	
	void ObstacleAvoidance3::freeze() {
		//Zero all components of the linear and angular velocity
		geometry_msgs::TwistStamped cmdvel_command_msg;
	
		ros::Time my_time = ros::Time::now();
			
		cmdvel_command_msg.header.stamp = my_time;
		cmdvel_command_msg.twist.linear.x = 0;
		cmdvel_command_msg.twist.linear.y = 0;
		cmdvel_command_msg.twist.linear.y = 0;
		cmdvel_command_msg.twist.linear.z = 0;
		cmdvel_command_msg.twist.angular.x = 0;
		cmdvel_command_msg.twist.angular.y = 0;
		cmdvel_command_msg.twist.angular.z = 0;
			
		cmdvel_pub.publish(cmdvel_command_msg);
	}
	
	bool ObstacleAvoidance3::emergencyCheck() {
		//Check the reported distances and if discovered that an obstacle is
		//close enough send a freeze
		if (frontDistance < EMERGENCY_DIST  ||
			leftDistance < EMERGENCY_DIST || rightDistance < EMERGENCY_DIST) {
				ROS_INFO("Emergency sistuation - land immediately...\n");
				freeze();
				return true;
			}
			
		return false;
	}
			
	void ObstacleAvoidance3::lift(float height) {
		//perform a lift towards the desired height
		//of a certain duration
		ros::Duration liftDuration(1.0);
		
		geometry_msgs::Point liftToPoint;
		
		liftToPoint.z = height;
		while ((ros::Time::now() - lastSwitch) < liftDuration){
			hover_pub.publish(liftToPoint);
		}
		
		ROS_INFO("Lift performed...\n");
	}
	
	void ObstacleAvoidance3::taxi(char mode) {
		//This method moves the UAV at the time when a maneuver is take place
		geometry_msgs::TwistStamped cmdvel_command_msg;
		
		ros::Time my_time = ros::Time::now();
			
		cmdvel_command_msg.header.stamp = my_time;
		cmdvel_command_msg.twist.linear.x = MANEUVER_VELOCITY;
		
		float alignDistance = ALIGN_DISTANCE;
		float overcomeDistance = 2*MIN_FRONT_DIST;
		ros::Duration alignDuration(ALIGN_DISTANCE/MANEUVER_VELOCITY);
		ros::Duration overcomeDuration(overcomeDistance/MANEUVER_VELOCITY);
		ros::Time startTime = ros::Time::now();
		
		//if there is no forced movement just move forward as long as 
		//there is this command
		if (mode == 'n') {
			cmdvel_pub.publish(cmdvel_command_msg);
			ROS_INFO("Taxi drive...\n");
		} else if (mode == 'y') {
			//if you are in the mode of forced taxi move forward
			//for certain duration as losn as you cover the requested distance
			while ((ros::Time::now() - startTime) < alignDuration) {
				cmdvel_pub.publish(cmdvel_command_msg);
				ROS_INFO("Forced Taxi drive...align\n");
				if (true == emergencyCheck()) {
					state = 3;
				}
			}
		} else if (mode == 'h') {
			//if you perform an overcome of an obstacle move as the previous option
			//but for a different distance
			while ((ros::Time::now() - startTime) < overcomeDuration) {
				cmdvel_pub.publish(cmdvel_command_msg);
				ROS_INFO("Forced Taxi drive...overcome\n");
				if (true == emergencyCheck()) {
					state = 3;
				}
			}
		}
	}
	
	
	void ObstacleAvoidance3::imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg) {
		//IMU callback function
		//It reports a quaternion, then this is converted to RPY rotations
		//and the current yaw is reported as the heading of the UAV
		
		double cur_roll, cur_pitch, cur_yaw;
		
		tf::Quaternion bq(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
		
		tf::Matrix3x3(bq).getRPY(cur_roll, cur_pitch, cur_yaw);
		
		currentHeading = cur_yaw;
		
		//ROS_INFO("%f\n", currentHeading*(180/M_PI));
	
	}
	
	void ObstacleAvoidance3::sonarHeightCallBack(const std_msgs::Float32::ConstPtr& heightMsg) {
		//Sonar to measure height
		//callback function to report the curent height
		currentHeight = heightMsg->data;
		
		//ROS_INFO("%f\n", currentHeight);
	}

	void ObstacleAvoidance3::laser0CallBack(const std_msgs::Float32::ConstPtr& scan0_msg) {
		//laser sensor measures the distance in 0 heading
		//callback function to report the front distance
		if (0 == scan0_msg->data) {
			frontDistance = MAX_LASER_DIST;
		} else {
			frontDistance = scan0_msg->data;
		}
	}

	void ObstacleAvoidance3::laser90CallBack(const std_msgs::Float32::ConstPtr& scan90_msg) {
		//laser sensor measures the distance in 90 heading
		//callback function to report the left distance
		if (0 == scan90_msg->data) {
			leftDistance = MAX_LASER_DIST;
		} else {
			leftDistance = scan90_msg->data;
		}
	}

	void ObstacleAvoidance3::laserm90CallBack(const std_msgs::Float32::ConstPtr& scanm90_msg) {
		//laser sensor measures the distance in -90 heading
		//callback function to report the right distance
		if (0 == scanm90_msg->data) {
			rightDistance = MAX_LASER_DIST;
		} else {
			rightDistance = scanm90_msg->data;
		}
	}
	

int main(int argc, char** argv) {
	ros::init(argc, argv, "ObstacleAvoidance3_node");
	
	ObstacleAvoidance3 obsAvd2;
	obsAvd2.state = 0;
	double liftToHeight;
	bool fail = false;
	bool driveInsurance = true;
	char turn;
	
	//start the actionlib server responsible for changing the heading of the UAV
	actionlib::SimpleActionClient<my_px4::changeHeadingAction> chHeadSrv("changeHeading", true);
  
	ROS_INFO("Waiting for changeHeading server to start.");
	chHeadSrv.waitForServer();
	ROS_INFO("changeHeading server started, ready to receive goal.");
	my_px4::changeHeadingGoal goalHeading;
	
	//start the actionlib server responsible for changing the hover altitude of the UAV
	actionlib::SimpleActionClient<my_px4::hoverHeightAction> hvHeightSrv("hoverHeight", true);
	
	ROS_INFO("Waiting for hoverHeight server to start.");
	hvHeightSrv.waitForServer();
	ROS_INFO("hoverHeight server started, sending goal.");
	my_px4::hoverHeightGoal goalHeight;
	
	//elevate to the desired heading altitude
	goalHeight.height = HOVER_HEIGHT;
	hvHeightSrv.sendGoal(goalHeight);
	
	bool finished_before_timeout = hvHeightSrv.waitForResult(ros::Duration(30.0));
						
	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = hvHeightSrv.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	} else {
		ROS_INFO("Action did not finish before the time out.");
	}
	
	//loop throught the finite state machine at a given rate of 20Hz
	ros::Rate r(20);
	while (ros::ok()) {
		
		switch(obsAvd2.state) {
			case 0:
				
				// reset the flag of reaching the maximum height without having found
				// the required clearence in front
				fail = false;
				
				if (obsAvd2.frontDistance < MIN_FRONT_DIST && obsAvd2.frontDistance > MIN_LASER_DIST) {
					obsAvd2.state = 3;
				}
				break;							
			case 3:
				// In case that UAV is in emergency situation - just land!
				goalHeight.height = 0;
				hvHeightSrv.sendGoal(goalHeight);
				
				{		
					//wait for the action to return		
					bool finished_before_timeout = hvHeightSrv.waitForResult(ros::Duration(30.0));
			
					if (finished_before_timeout) {
						actionlib::SimpleClientGoalState state = hvHeightSrv.getState();
						ROS_INFO("Action finished: %s",state.toString().c_str());
					} else {
						ROS_INFO("Action did not finish before the time out.");
					}
				}
				
				break;
			
			default: 
				// Catch possible errors
				ROS_INFO("No state\n");
				break;
			}
		
		//ROS_INFO("The state is:%d\n", obsAvd2.state);
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
	
	
	
