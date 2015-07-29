////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  Actionlib server responsible for moving the UAV	      			   
//	a requested vertical distance based on the GPS measurement        
//  (altitude)	  													  
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_hector/liftDistanceAction.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <math.h>

class liftDistanceAction
{
  
  public:
  
  liftDistanceAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&liftDistanceAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&liftDistanceAction::preemptCB, this));

    //publish to cmd_vel topic to change the height
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //subscribe to the data topic transmitted by the GPS
	gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("/fix", 1, &liftDistanceAction::gpsCallBack, this);
    //start the server
    as_.start();
    
    //usefull flag to mark the initial height
    firstTime = true;
  }

  ~liftDistanceAction(void)
  {
  }
  
  void goalCB()
  {
    // accept the new goal
    desiredDistance = as_.acceptNewGoal()->distance;
    ROS_INFO("%s: New Goal accepted: %f", action_name_.c_str(), desiredDistance);
  }
  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
  
  void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg) {
	
	// make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
     
	double currentAltitude;
	double altitudeDifference;
	double gainP, linearVelocityZ;
	
	//get the current altitude	
	currentAltitude = gpsMsg->altitude;
	
	//register the value of the altitude at the beginning of the task
	if (firstTime) {
		desiredAltitude = currentAltitude + desiredDistance;
		firstTime = false;
	}
	
	gainP = 0.5;
	
	altitudeDifference = desiredAltitude - currentAltitude;
	
	//report the feedback (the remaining difference)
	feedback_.distanceTotal = altitudeDifference;
	
	ROS_INFO("Remaining Diference in Altitude: %f\n", feedback_.distanceTotal);
	
	//implement a simple P-controller
	linearVelocityZ = gainP*altitudeDifference;
		
	geometry_msgs::Twist cmd;
	//prepare the command with the appropriate linear velocity component
	cmd.linear.z = linearVelocityZ;
	
	//condition to stop and return SUCCESS
	//request high accuracy given the measurement devices' resolution		
	if (altitudeDifference < 0.1) {
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		cmd.linear.z = 0;
        // set the action state to succeeded
        result_.distanceTraversed = desiredDistance;
        as_.setSucceeded(result_);
	}
	
	vel_pub.publish(cmd);
  }
  
  protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<my_hector::liftDistanceAction> as_;
  std::string action_name_;
  my_hector::liftDistanceFeedback feedback_;
  my_hector::liftDistanceResult result_;
  ros::Subscriber gps_sub;
  ros::Publisher vel_pub;
  double desiredDistance;
  double desiredAltitude;
  bool firstTime;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "liftDistance");

  liftDistanceAction liftDistance(ros::this_node::getName());
  ros::spin();

  return 0;
}
