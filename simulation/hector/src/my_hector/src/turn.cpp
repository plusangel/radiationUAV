////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  UAV change heading												   
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <stdio.h>
#include <math.h>

using namespace std;

class Turn{
	public:
		Turn();
		double desiredHeading;
	private:
		ros::NodeHandle n; //create a handle for ROS node
		
		//subscribers
		ros::Subscriber imu_sub, turn_sub;
		//publisher
		ros::Publisher vel_pub;
		
		//callback functions of the subscribers
		void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg);
		void headingCallBack(const geometry_msgs::Quaternion::ConstPtr& headingMsg);
	};
	
	Turn::Turn() {
		//advertise the publisher's topics in the ROS using the master
		vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		//subscribe to the related topics
		imu_sub = n.subscribe<sensor_msgs::Imu>("/raw_imu", 1, &Turn::imuCallBack, this);
		turn_sub = n.subscribe<geometry_msgs::Quaternion>("/heading", 1, &Turn::headingCallBack, this);
	}
	
	void Turn::headingCallBack(const geometry_msgs::Quaternion::ConstPtr& headingMsg) {
		//listen for quaternion messages containing the desired change of the heading
		double des_roll, des_pitch, des_yaw;
		tf::Quaternion bq(headingMsg->x, headingMsg->y, headingMsg->z, headingMsg->w);
		//transform to RPY
		tf::Matrix3x3(bq).getRPY(des_roll, des_pitch, des_yaw);
		
		desiredHeading = des_yaw;
		//ROS_INFO("Roll: %f Pitch: %f Yaw: %f \n", des_roll*(180/M_PI), des_pitch*(180/M_PI), des_yaw*(180/M_PI));
	}
	
	void Turn::imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg) {
		
		double cur_roll, cur_pitch, cur_yaw;
		double currentHeading, headingDifference;
		double gainP, angularVelocityZ;
		
		tf::Quaternion bq(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
		
		//transform to RPY
		tf::Matrix3x3(bq).getRPY(cur_roll, cur_pitch, cur_yaw);	
		
		gainP = 0.5;
		
		//get the current heading
		currentHeading = cur_yaw;
		headingDifference = desiredHeading - currentHeading;
		
		//implement a simple P-controller
		angularVelocityZ = gainP*headingDifference;
		
		geometry_msgs::Twist cmd;
		//prepare the appropriate angular velocity component
		cmd.angular.z = angularVelocityZ;
		
		//ROS_INFO("%f %f\n", hoverMsg->range, verticalVelocityZ);
		
		vel_pub.publish(cmd);
		
		//ROS_INFO("Roll: %f Pitch: %f Yaw: %f \n", cur_roll*(180/M_PI), cur_pitch*(180/M_PI), cur_yaw*(180/M_PI));
			
	}
		
int main(int argc, char** argv) {
	ros::init(argc, argv, "turn_node");
	
	Turn turn;
	ros::spin();
}
