////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  Simple TwistStamped control command									    
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "fw_publisher");
	
	ros::NodeHandle n;
	ros::Publisher fwd_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 50);
	
	ros::Rate loop_rate(10);
	while (n.ok()) {
		ros::Time my_time = ros::Time::now();
		
		geometry_msgs::TwistStamped cmd;
		cmd.header.stamp = my_time;
		
		cmd.twist.linear.x = 0.0;
		cmd.twist.linear.y = 0.0;
		cmd.twist.linear.z = 0.5;
		cmd.twist.angular.x = 0.0;
		cmd.twist.angular.y = 0.0;
		cmd.twist.angular.z = 0.0;
		
		fwd_pub.publish(cmd);
		loop_rate.sleep();
	}
	
	return 0;
}
