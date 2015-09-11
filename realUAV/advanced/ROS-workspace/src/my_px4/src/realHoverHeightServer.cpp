////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring   
//  networks                              
//                                    
//  real UAV hover height actionlib                      
//                                    
//  Angelos Plastropoulos                         
////////////////////////////////////////////////////////////////////////#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <my_px4/hoverHeightAction.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>
#include <math.h>

#define SONAR_MOUNT_HEIGHT	0.1f

class hoverHeightAction
{
  
  public:
  
  hoverHeightAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&hoverHeightAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&hoverHeightAction::preemptCB, this));

    //publish to hover_height topic to request the change of the hover height
    hover_pub = nh_.advertise<geometry_msgs::Point>("/hover_height", 1);
    //subscribe to the data topic transmitted by the sonar responible to report the distance from the ground
    sonar_sub = nh_.subscribe<std_msgs::Float32>("/sonar", 1, &hoverHeightAction::sonarCallBack, this);
    //start the server
    as_.start();
    
    ROS_INFO("%s: Server is up&running", action_name_.c_str());
    
  }

  ~hoverHeightAction(void)
  {
  }
  
  void goalCB()
  {
    // accept the new goal
    desiredAltitude = as_.acceptNewGoal()->height;
    ROS_INFO("%s: New Goal accepted: %f", action_name_.c_str(), desiredAltitude);
  }
  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
  
  void sonarCallBack(const std_msgs::Float32::ConstPtr& sonarMsg) {
	
	// make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    geometry_msgs::Point liftToPoint;
     
	double currentAltitude;
	double altitudeDifference;
	
	//get the current height
	currentAltitude = sonarMsg->data;
	
	altitudeDifference = desiredAltitude - currentAltitude;
	
	liftToPoint.z = desiredAltitude;
	
	//report the feedback (the remaining difference)
	feedback_.heightCurrent = currentAltitude;
	
	ROS_INFO("Current Altitude: %f\n", feedback_.heightCurrent);
	
	//condition to stop and return SUCCESS
	//request high accuracy given the measurement devices' resolution
	if (abs(altitudeDifference) < 0.1) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		// cmd.linear.z = 0;
        // set the action state to succeeded
        result_.heightDifference = altitudeDifference;
        as_.setSucceeded(result_);
	}
	
	hover_pub.publish(liftToPoint);	
	
  }
  
  protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_px4::hoverHeightAction> as_;
  std::string action_name_;
  my_px4::hoverHeightFeedback feedback_;
  my_px4::hoverHeightResult result_;
  ros::Subscriber sonar_sub;
  ros::Publisher hover_pub;
  double desiredDistance;
  double desiredAltitude;
  bool firstTime;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "realHoverHeight");

  hoverHeightAction hoverHeight(ros::this_node::getName());
  ros::spin();

  return 0;
}
