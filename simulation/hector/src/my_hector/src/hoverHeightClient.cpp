////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  Test the actionlib server responsible for changing the UAV         
//	altitude to a requested height									  
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_hector/hoverHeightAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_hoverHeight");

  //create the action client
  actionlib::SimpleActionClient<my_hector::hoverHeightAction> ac("hoverHeight");
  
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  //send a goal to the action
  my_hector::hoverHeightGoal goal;
  //move to altitude equal to 1m
  goal.height = 1.0;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
