////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  //
//  networks														  //
//  																  //
//  Actionlib server responsible for changing the UAV heading		  //  
//																	  //
//  Angelos Plastropoulos 											  //
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_hector/changeHeadingAction.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <stdio.h>
#include <math.h>

class changeHeadingAction
{
  
  
  public:
  
  changeHeadingAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&changeHeadingAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&changeHeadingAction::preemptCB, this));

	//publish to cmd_vel topic to control the heading using the appropriate component of the angular
	//velocity 
	vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //subscribe to the data IMU topic to obtain the required quaternion
	imu_sub = nh_.subscribe<sensor_msgs::Imu>("/raw_imu", 1, &changeHeadingAction::imuCallBack, this);
	//start the server
    as_.start();
    
    ROS_INFO("%s: Server is up&running", action_name_.c_str());
  }

  ~changeHeadingAction(void)
  {
  }
  
  void goalCB()
  {
    //accept the new goal
    desiredHeading = as_.acceptNewGoal()->angle;
    ROS_INFO("%s: New Goal accepted: %f", action_name_.c_str(), desiredHeading);
  }
  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    //set the action state to preempted
    as_.setPreempted();
  }
  
  void imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg) {
	
	//make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
     
  
	double cur_roll, cur_pitch, cur_yaw;
	double currentHeading, headingDifference;
	double gainP, angularVelocityZ;
	
	//get the current quaternion from the IMU	
	tf::Quaternion bq(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
	//convert it to RPY	
	tf::Matrix3x3(bq).getRPY(cur_roll, cur_pitch, cur_yaw);	
		
	gainP = 0.5;
	//obtain the yaw rotation
	currentHeading = cur_yaw;
	
	headingDifference = desiredHeading - currentHeading;
	//report the feedback (the remaining difference)
	feedback_.angleTotal = headingDifference;
	
	ROS_INFO("Remaining Diference in Heading: %f\n", feedback_.angleTotal*(180/M_PI));
	
	//implement a simple P-controller
	angularVelocityZ = gainP*headingDifference;
	
	//issue the required angular velocity	
	geometry_msgs::Twist cmd;
	cmd.angular.z = angularVelocityZ;
			
	vel_pub.publish(cmd);
	
	//condition to stop and return SUCCESS
	//consider a small margin to avoid overcome	
	if (abs(headingDifference*(180/M_PI)) < 3) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        result_.angleTurned = desiredHeading;
        as_.setSucceeded(result_);
	}
  }
  
  protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_hector::changeHeadingAction> as_;
  std::string action_name_;
  my_hector::changeHeadingFeedback feedback_;
  my_hector::changeHeadingResult result_;
  double desiredHeading;
  ros::Subscriber imu_sub;
  ros::Publisher vel_pub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "changeHeading");

  changeHeadingAction changeHeading(ros::this_node::getName());
  ros::spin();

  return 0;
}
