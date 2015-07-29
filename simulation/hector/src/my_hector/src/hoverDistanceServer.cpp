////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  Actionlib server responsible for moving the UAV	        		   
//	a requested vertical distance based on the sonar measurement      
//  (distance from ground)	  										  
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_hector/hoverDistanceAction.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/Range.h>
#include <geometry_msgs/Point.h>

#include <stdio.h>
#include <math.h>

#define SONAR_MOUNT_HEIGHT	0.1f //consider the height in which the sonar
								 //is mounted to enhance the calculations
								 //accuracy

class hoverDistanceAction
{
  
  public:
  
  hoverDistanceAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&hoverDistanceAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&hoverDistanceAction::preemptCB, this));

    //publish to hover_height topic to request the change of the hover height 
    hover_pub = nh_.advertise<geometry_msgs::Point>("/hover_height", 1);
    //subscribe to the data topic transmitted by the sonar responible to report the distance from the ground
	sonar_sub = nh_.subscribe<sensor_msgs::Range>("/sonar_height", 1, &hoverDistanceAction::sonarCallBack, this);
    //start the server
    as_.start();
    
    ROS_INFO("%s: Server is up 'n' running", action_name_.c_str());
    
    //usefull flag to mark the initial height
    firstTime = true;
  }

  ~hoverDistanceAction(void)
  {
  }
  
  void goalCB()
  {
    //accept the new goal
    desiredDistance = as_.acceptNewGoal()->distance;
    ROS_INFO("%s: New Goal accepted: %f", action_name_.c_str(), desiredDistance);
  }
  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    //set the action state to preempted
    as_.setPreempted();
  }
  
  void sonarCallBack(const sensor_msgs::Range::ConstPtr& sonarMsg) {
	
	//make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    geometry_msgs::Point liftToPoint;
     
	double currentAltitude;
	double altitudeDifference;
		
	//get the current altitude	
	currentAltitude = sonarMsg->range;
	
	//register the value of the altitude at the beginning of the task
	if (firstTime) {
		desiredAltitude = currentAltitude + desiredDistance - SONAR_MOUNT_HEIGHT;
		firstTime = false;
	}
	
	//calculate the difference
	altitudeDifference = desiredAltitude - currentAltitude;
	
	//set the desired altitude
	liftToPoint.z = desiredAltitude;
	
	//report the feedback (the remaining difference)
	feedback_.distanceTotal = altitudeDifference;
	
	ROS_INFO("Altitude difference to goal: %f\n", feedback_.distanceTotal);
	
	//condition to stop and return SUCCESS
	//request high accuracy given the measurement devices' resolution	
	if (abs(altitudeDifference) < 0.1) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
        result_.distanceTraversed = desiredDistance;
        as_.setSucceeded(result_);
        firstTime = true;
	}
	
	hover_pub.publish(liftToPoint);	
	
  }
  
  protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_hector::hoverDistanceAction> as_;
  std::string action_name_;
  my_hector::hoverDistanceFeedback feedback_;
  my_hector::hoverDistanceResult result_;
  ros::Subscriber sonar_sub;
  ros::Publisher hover_pub;
  double desiredDistance;
  double desiredAltitude;
  bool firstTime;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hoverDistance");

  hoverDistanceAction hoverDistance(ros::this_node::getName());
  ros::spin();

  return 0;
}
