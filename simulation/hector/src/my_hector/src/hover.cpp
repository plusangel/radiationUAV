////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  UAV hover (Altitude Hold mode)									    
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>

#define MAX_HOVER_HEIGHT  3.0f //max permissible hover altitude

using namespace std;

class Hover{
	public:
		Hover();
		double desiredHeight;
	private:
		ros::NodeHandle n; //create a handle for ROS node
		
		//subscribers
		ros::Subscriber sonar_sub, hover_sub;
		//publisher
		ros::Publisher vel_pub;
		
		//callback functions of the subscribers
		void sonarCallBack(const sensor_msgs::Range::ConstPtr& hoverMsg);
		void heightCallBack(const geometry_msgs::Point::ConstPtr& heightMsg);
	};
	
	Hover::Hover() {
		
		//advertise the publisher's topics in the ROS using the master
		vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		//subscribe to the related topics
		sonar_sub = n.subscribe<sensor_msgs::Range>("/sonar_height", 1, &Hover::sonarCallBack, this);
		hover_sub = n.subscribe<geometry_msgs::Point>("/hover_height", 1, &Hover::heightCallBack, this);
	}
	
	void Hover::heightCallBack(const geometry_msgs::Point::ConstPtr& heightMsg) {
		//listen for point messages containing the desired altitude to lock
		//check if you are inside the permissible limits
		if (heightMsg->z <= MAX_HOVER_HEIGHT) {
			
			desiredHeight = heightMsg->z;
			ROS_INFO("%f\n", desiredHeight);
		} else {
			ROS_INFO("Cannot hover above %f\n", MAX_HOVER_HEIGHT);
		}
			
		
	}
	
	void Hover::sonarCallBack(const sensor_msgs::Range::ConstPtr& hoverMsg) {
		//measure the distance from the ground and create vertical movement
		//in order to adjust height according to the desired altitude lock
		double currentHeight, heightDifference;
		double gainP, verticalVelocityZ;
		
		gainP = 0.5;
		
		//get the current height
		currentHeight = hoverMsg->range;
		heightDifference = desiredHeight - currentHeight;
		
		//implement a simple P-controller
		verticalVelocityZ = gainP*heightDifference;
		
		geometry_msgs::Twist cmd;
		
		//prepare the appropriate vertical velocity component
		cmd.linear.z = verticalVelocityZ;
		
		//ROS_INFO("%f %f\n", hoverMsg->range, verticalVelocityZ);
		
		vel_pub.publish(cmd);
		
	}
		
int main(int argc, char** argv) {
	ros::init(argc, argv, "hover_node");
	
	Hover hover;
	ros::spin();
}
