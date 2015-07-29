////////////////////////////////////////////////////////////////////////
//  Next generation mapping using UAS assisted dynamic monitoring	  
//  networks														  
//  																  
//  Obstacle Avoidance Finate State Machine (version.2)				   
//																	  
//  Angelos Plastropoulos 											  
////////////////////////////////////////////////////////////////////////

// Most common ros headers
#include <ros/ros.h>

// Action headers
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_hector/changeHeadingAction.h>
#include <my_hector/hoverHeightAction.h>
#include <boost/thread.hpp>

// Messages headers
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Twist.h>
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
#define MAX_HOVER_HEIGHT    3.0f //this is the limit of the sensor which report
								 //the distance from the ground
#define MAX_MANEUVER_HEIGHT 2.5f //do not allow vertical maneuver over that
#define HOVER_HEIGHT		1.0f //the altitude that you set for the hover
#define DEADEND_DISTANCE    5.0f //this is the limit where you decide that you
								 //in deadend (distance in the left and right)
#define MIN_LASER_DIST  	0.3f //this is the base of the UAV which reaported as obstacle
								 //from the laser
#define MAX_LASER_DIST      30.0f//the max range of the laser
#define MIN_FRONT_DIST 		2.0f //under that limit an object is sensed in front
#define EMERGENCY_DIST		1.0f //do not allow movement if sence object closer
								 //to this

using namespace std;

class ObstacleAvoidance2 {
	public:
		ObstacleAvoidance2();
		
		void taxi(char mode); //taxi mode
		void lift(float height); //lift to a height
		void freeze(); //freeze
		
		//state of the FSM
		int state;
		ros::Time lastSwitch;
		//distances around the UAV
		double backDistance, frontDistance;
		double leftDistance, rightDistance;
		double currentHeight, currentHeading;
	
	private:
		ros::NodeHandle n; //create a handle for the ROS node
		//list of subscribers
		ros::Subscriber lscan_sub, sscan_sub, cmdvel_sub, sheight_sub, imu_sub; 
		//list of publishers
		ros::Publisher cmdvel_pub, hover_pub;// heading_pub;
		
		//callback functions of the subscribers
		void laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan_msg);
		void sonarBackCallBack(const sensor_msgs::Range::ConstPtr& sscan_msg);
		void cmdvelGetawayCallBack(const geometry_msgs::Twist::ConstPtr& cmdvel_msg);
		void sonarHeightCallBack(const sensor_msgs::Range::ConstPtr& heightMsg);
		void imuCallBack(const sensor_msgs::Imu::ConstPtr& heightMsg);
		
		//helper methods
		float getFrontDistance(vector<float> myScans, int range);
		float getSideDistance(vector<float> myScans, int range, char side);
		bool emergencyCheck();
	};
	
	ObstacleAvoidance2::ObstacleAvoidance2() {
		
		//subscribe to the related topics 
		lscan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, &ObstacleAvoidance2::laserScanCallBack, this);
		sscan_sub = n.subscribe<sensor_msgs::Range>("/sonar_back", 100, &ObstacleAvoidance2::sonarBackCallBack, this);
		cmdvel_sub = n.subscribe<geometry_msgs::Twist>("/obs_cmd_vel", 100, &ObstacleAvoidance2::cmdvelGetawayCallBack, this);
		sheight_sub = n.subscribe<sensor_msgs::Range>("/sonar_height", 100, &ObstacleAvoidance2::sonarHeightCallBack, this);
		imu_sub = n.subscribe<sensor_msgs::Imu>("/raw_imu", 1, &ObstacleAvoidance2::imuCallBack, this);
		
		//advertise the publisher's topics in the ROS using the master
		cmdvel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
		hover_pub = n.advertise<geometry_msgs::Point>("/hover_height", 100);
		//heading_pub = n.advertise<geometry_msgs::Quaternion>("/heading", 100);
	}
	
	void ObstacleAvoidance2::cmdvelGetawayCallBack(const geometry_msgs::Twist::ConstPtr& cmdvel_msg) {
		//if there is no obstacle keep transmitting what received from the imputs (keyboard or joystic)
		//to the normal topic for movement 'cmd_vel'. This is the manual mode - no intervation
		//If for any reason sense something (emergencyChack) closer than 1m go to freeze
		geometry_msgs::Twist cmdvel_command_msg;
	
		if (state == 0) {
			
			cmdvel_command_msg.linear.x = cmdvel_msg->linear.x;
			cmdvel_command_msg.linear.y = cmdvel_msg->linear.y;
			cmdvel_command_msg.linear.z = cmdvel_msg->linear.z;
			cmdvel_command_msg.angular.x = cmdvel_msg->angular.x;
			cmdvel_command_msg.angular.y = cmdvel_msg->angular.y;
			cmdvel_command_msg.angular.z = cmdvel_msg->angular.z;
			
			cmdvel_pub.publish(cmdvel_command_msg);
			
			ROS_INFO("Manual mode...\n");
			
			if (true == emergencyCheck()) {
				state = 3;
			}
		} 
	}
	
	void ObstacleAvoidance2::freeze() {
		//Zero all components of the linear and angular velocity
		geometry_msgs::Twist cmdvel_command_msg;
	
		cmdvel_command_msg.linear.x = 0;
		cmdvel_command_msg.linear.y = 0;
		cmdvel_command_msg.linear.y = 0;
		cmdvel_command_msg.linear.z = 0;
		cmdvel_command_msg.angular.x = 0;
		cmdvel_command_msg.angular.y = 0;
		cmdvel_command_msg.angular.z = 0;
			
		cmdvel_pub.publish(cmdvel_command_msg);
	}
	
	bool ObstacleAvoidance2::emergencyCheck() {
		//Check the reported distances and if discovered that an obstacle is
		//close enough send a freeze
		if (frontDistance < EMERGENCY_DIST || backDistance < EMERGENCY_DIST ||
			leftDistance < EMERGENCY_DIST || rightDistance < EMERGENCY_DIST) {
				ROS_INFO("Emergency sistuation - land immediately...\n");
				freeze();
				return true;
			}
			
		return false;
	}
			
	void ObstacleAvoidance2::lift(float height) {
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
	
	void ObstacleAvoidance2::taxi(char mode) {
		//This method moves the UAV at the time when a maneuver is take place
		geometry_msgs::Twist cmdvel_command_msg;
		cmdvel_command_msg.linear.x = MANEUVER_VELOCITY;
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
	
	
	void ObstacleAvoidance2::imuCallBack(const sensor_msgs::Imu::ConstPtr& imuMsg) {
		//IMU callback function
		//It reports a quaternion, then this is converted to RPY rotations
		//and the current yaw is reported as the heading of the UAV
		
		double cur_roll, cur_pitch, cur_yaw;
		
		tf::Quaternion bq(imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
		
		tf::Matrix3x3(bq).getRPY(cur_roll, cur_pitch, cur_yaw);
		
		currentHeading = cur_yaw;
		
		//ROS_INFO("%f\n", currentHeading*(180/M_PI));
	
	}
	
	void ObstacleAvoidance2::sonarHeightCallBack(const sensor_msgs::Range::ConstPtr& heightMsg) {
		//Sonar to measure height
		//callback function to report the curent height
		currentHeight = heightMsg->range;
		
		//ROS_INFO("%f\n", currentHeight);
	}
	
	void ObstacleAvoidance2::sonarBackCallBack(const sensor_msgs::Range::ConstPtr& sscan_msg) {
		//Sonar to measure the back distance
		//callback function to report the distance available in the back of the UAV
		backDistance = sscan_msg->range;
		
		//ROS_INFO("Back distance:%f\n", backDist);
	}
	
    void ObstacleAvoidance2::laserScanCallBack(const sensor_msgs::LaserScan::ConstPtr& lscan_msg) {
		//Laser callback function
		//It gets the laserScan message (180 degrees)
		//Using helper methods it estimates the distances
		//to front and both sides
		int numOfScans = lscan_msg->ranges.size();
		float minAngle = (lscan_msg->angle_min)*(180/M_PI);
		float maxAngle = (lscan_msg->angle_max)*(180/M_PI);
		vector<float> laserScannings = lscan_msg->ranges;
		
		frontDistance = ObstacleAvoidance2::getFrontDistance(laserScannings, 40);
		leftDistance = ObstacleAvoidance2::getSideDistance(laserScannings, 20, 'l');
		rightDistance = ObstacleAvoidance2::getSideDistance(laserScannings, 20, 'r');
		
	}	
	
	float ObstacleAvoidance2::getFrontDistance(vector<float> myScans, int range) {
		//It returns the minimum distance in front taking into account
		//a range of degrees in the front (range degrees angle centered in 0 heading)
		int numOfScannings, frontHeading;
		float minFrontDist = 100;
		
		numOfScannings = myScans.size();
		frontHeading = numOfScannings/2.0;
		
		//ROS_INFO("Vec size:%d frontHeading:%d \n", numOfScannings, frontHeading);
		
		for(int i = (frontHeading - range/2.0); i < (frontHeading + range/2.0); i++) {
			if(myScans[i] <= MIN_LASER_DIST) {
				continue;
			}else if (myScans[i] < minFrontDist) {
				minFrontDist = myScans[i];
			}
		}
		
		//ROS_INFO("Min front dist:%f\n", minFwDist);
		return minFrontDist;
	}
	
	float ObstacleAvoidance2::getSideDistance(vector<float> myScans, int range, char side) {
		//It returns the minimum distance on a side taking into account an 
		//angle of range degrees
		int numOfScannings, sideHeading;
		float minSideDist = 100;
		
		numOfScannings = myScans.size();
		
		if(side == 'l') {
			sideHeading = 0;
		} else if(side == 'r') {
			sideHeading = (numOfScannings - range);
		} else {
			ROS_INFO("Incorrect direction setting\n");
			exit;
		}
		
		//ROS_INFO("Vec size:%d frontHeading:%d \n", numOfScannings, frontHeading);
		
		for(int i = sideHeading; i < (sideHeading + range); i++) {
			if(myScans[i] <= MIN_LASER_DIST) {
				continue;
			} else if(myScans[i] < minSideDist) {
				minSideDist = myScans[i];
			}
		}
		
		//ROS_INFO("Min side dist:%f\n", minSideDist);
		return minSideDist;
	}
			
int main(int argc, char** argv) {
	ros::init(argc, argv, "ObstacleAvoidance2_node");
	
	ObstacleAvoidance2 obsAvd2;
	obsAvd2.state = 0;
	double liftToHeight;
	bool fail = false;
	bool driveInsurance = true;
	char turn;
	
	//start the actionlib server responsible for changing the heading of the UAV
	actionlib::SimpleActionClient<my_hector::changeHeadingAction> chHeadSrv("changeHeading", true);
  
	ROS_INFO("Waiting for changeHeading server to start.");
	chHeadSrv.waitForServer();
	ROS_INFO("changeHeading server started, ready to receive goal.");
	my_hector::changeHeadingGoal goalHeading;
	
	//start the actionlib server responsible for changing the hover altitude of the UAV
	actionlib::SimpleActionClient<my_hector::hoverHeightAction> hvHeightSrv("hoverHeight", true);
	
	ROS_INFO("Waiting for hoverHeight server to start.");
	hvHeightSrv.waitForServer();
	ROS_INFO("hoverHeight server started, sending goal.");
	my_hector::hoverHeightGoal goalHeight;
	
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
					// if the space in front is less than the required clerance (don't get mislead from the
					// distances measured because of the UAV base
					//ROS_INFO("1.righ:%f left:%f front:%f\n", obsAvd2.rightDistance, obsAvd2.leftDistance, obsAvd2.frontDistance);
					ROS_INFO("1.Obstacle in front\n");
					
					if (obsAvd2.rightDistance < DEADEND_DISTANCE && obsAvd2.leftDistance < DEADEND_DISTANCE) {
						// if the distances in left and right are less than the dead end threshold
						// the UAV need to move up
						
						obsAvd2.state = 2;
						ROS_INFO("Too narrow - Going up\n");
					} else if (obsAvd2.rightDistance > obsAvd2.leftDistance && obsAvd2.rightDistance > DEADEND_DISTANCE) {
						// if the distance in right is greater than in left, turn right
						//obsAvd2.lastSwitch = ros::Time::now();
						turn = 'r';
						
						//obsAvd2.changeHeading(angle);
						// turn M_PI/2 from your heading counter-clockwise 
						goalHeading.angle = obsAvd2.currentHeading - M_PI/2;
						chHeadSrv.sendGoal(goalHeading);
						
						//wait for the action to return		
						bool finished_before_timeout = chHeadSrv.waitForResult(ros::Duration(30.0));
			
						if (finished_before_timeout) {
							actionlib::SimpleClientGoalState state = chHeadSrv.getState();
							ROS_INFO("Action finished: %s",state.toString().c_str());
						} else {
							ROS_INFO("Action did not finish before the time out.");
						}	
						
						// and change to the appropriate state
						obsAvd2.state = 1;
						ROS_INFO("Going right\n");
					} else if (obsAvd2.rightDistance < obsAvd2.leftDistance && obsAvd2.leftDistance > DEADEND_DISTANCE) {
						// if the distance in left is greater than in right, turn left
						//obsAvd2.lastSwitch = ros::Time::now();
						turn = 'l';
						
						//obsAvd2.changeHeading(angle);
						// turn M_PI/2 from your heading counter-clockwise 
						goalHeading.angle = obsAvd2.currentHeading + M_PI/2;
						chHeadSrv.sendGoal(goalHeading);
						
						//wait for the action to return		
						bool finished_before_timeout = chHeadSrv.waitForResult(ros::Duration(30.0));
			
						if (finished_before_timeout) {
							actionlib::SimpleClientGoalState state = chHeadSrv.getState();
							ROS_INFO("Action finished: %s",state.toString().c_str());
						} else {
							ROS_INFO("Action did not finish before the time out.");
						}
						
						// and change to the appropriate state
						obsAvd2.state = 1;
						ROS_INFO("Going left\n");
					} else if (obsAvd2.rightDistance == obsAvd2.leftDistance && obsAvd2.leftDistance == MAX_LASER_DIST) {
						// if both sides are clear turn left
						//obsAvd2.lastSwitch = ros::Time::now();
						turn = 'l';
						
						//obsAvd2.changeHeading(angle);
						// turn M_PI/2 from your heading counter-clockwise 
						goalHeading.angle = obsAvd2.currentHeading + M_PI/2;
						chHeadSrv.sendGoal(goalHeading);
						
						//wait for the action to return		
						bool finished_before_timeout = chHeadSrv.waitForResult(ros::Duration(30.0));
			
						if (finished_before_timeout) {
							actionlib::SimpleClientGoalState state = chHeadSrv.getState();
							ROS_INFO("Action finished: %s",state.toString().c_str());
						} else {
							ROS_INFO("Action did not finish before the time out.");
						}
						
						// and change to the appropriate state
						obsAvd2.state = 1;
						ROS_INFO("Equal - Going left\n");
					}
				}
				break;
			case 1:
				// move forward and check the related distances around to decide
				// the next step
				obsAvd2.taxi('n');
				//ROS_INFO("2.righ:%f left:%f front:%f\n", obsAvd2.rightDistance, obsAvd2.leftDistance, obsAvd2.frontDistance);
				
				if (obsAvd2.frontDistance < MIN_FRONT_DIST && obsAvd2.frontDistance > MIN_LASER_DIST && driveInsurance == true) {
					// in case that during the maneuver taxi find in front obstacle closer
					// to the safety distance stop to re-evaluate the maneuver 
					ROS_INFO("Emergency: Obstacle in front!\n");
					
					//disable this check till UAV change orientation otherwise
					// you will stuck
					driveInsurance = false;
					obsAvd2.state = 0;
				} else if (obsAvd2.leftDistance < MAX_CLEAR_DIST && turn == 'r') {
					// since the obstacle is present in the related direction keep on moving
					// and if front UAV has plenty of space enable the insurance (if it has been disabled
					// in previous step)
					
					if (obsAvd2.frontDistance > MIN_FRONT_DIST && obsAvd2.frontDistance > MIN_LASER_DIST) {
						driveInsurance = true;
					}
					obsAvd2.state = 1;
				} else if (obsAvd2.rightDistance < MAX_CLEAR_DIST && turn == 'l') {
					// since the obstacle is present in the related direction keep on moving
					// and if front UAV has plenty of space enable the insurance (if it has been disabled
					// in previous step)
					
					if (obsAvd2.frontDistance > MIN_FRONT_DIST && obsAvd2.frontDistance > MIN_LASER_DIST) {
						driveInsurance = true;
					}
					obsAvd2.state = 1;
				} else if ((obsAvd2.leftDistance > MAX_CLEAR_DIST && turn == 'r')  ||
				 (obsAvd2.rightDistance > MAX_CLEAR_DIST && turn == 'l')) {
					// success! UAV has reached the end of the obstacle
					// move forward a little bit to gain some space
					obsAvd2.taxi('y');  //forced taxi to align
					
					// turn back to the appropriate orientation as it
					// was before the maneuver
					if (turn == 'l') {
						goalHeading.angle = obsAvd2.currentHeading - M_PI/2;
					} else if (turn == 'r') {
						goalHeading.angle = obsAvd2.currentHeading + M_PI/2;
					}
					chHeadSrv.sendGoal(goalHeading);
						
					//wait for the action to return		
					bool finished_before_timeout = chHeadSrv.waitForResult(ros::Duration(30.0));
			
					if (finished_before_timeout) {
						actionlib::SimpleClientGoalState state = chHeadSrv.getState();
						ROS_INFO("Action finished: %s",state.toString().c_str());
					} else {
						ROS_INFO("Action did not finish before the time out.");
					}
					
					//obsAvd2.lastSwitch = ros::Time::now();
					//angle = 0;
					//obsAvd2.changeHeading(angle);
					//enable the insurance if it has been disebled in the past
					driveInsurance = true;
					obsAvd2.state = 0;
				}	
				break;
			case 2:
				// lift up till the maximum height that UAV can maneuver 
				// safely realated to the max hover height
				liftToHeight = MAX_MANEUVER_HEIGHT;
				
				// check if constraint implies and unless front clearence
				// has not failed move up
				if (liftToHeight <= MAX_HOVER_HEIGHT && fail == false) {
					obsAvd2.lastSwitch = ros::Time::now();
					obsAvd2.lift(liftToHeight);
				}
				
				//ROS_INFO("front:%f\n", obsAvd2.frontDistance);
				if (obsAvd2.frontDistance > MAX_CLEAR_DIST) {
					// In case that the front space is greater than the clearence 
					// Success!
					
					//obsAvd2.lastSwitch = ros::Time::now();
					//obsAvd2.lift(obsAvd2.currentHeight+0.5);
					// move to the specific altitude + 0.5 m to be safe
					goalHeight.height = obsAvd2.currentHeight+0.5;
					hvHeightSrv.sendGoal(goalHeight);
						
					//wait for the action to return		
					bool finished_before_timeout = hvHeightSrv.waitForResult(ros::Duration(30.0));
			
					if (finished_before_timeout) {
						actionlib::SimpleClientGoalState state = hvHeightSrv.getState();
						ROS_INFO("Action finished: %s",state.toString().c_str());
					} else {
						ROS_INFO("Action did not finish before the time out.");
					}	
					
					// move forward to overacome the height change
					obsAvd2.taxi('h');
					
					// correct back to the defined hover height to let
					// the operator continue
					goalHeight.height = HOVER_HEIGHT;
					hvHeightSrv.sendGoal(goalHeight);
						
					if (finished_before_timeout) {
						actionlib::SimpleClientGoalState state = hvHeightSrv.getState();
						ROS_INFO("Action finished: %s",state.toString().c_str());
					} else {
						ROS_INFO("Action did not finish before the time out.");
					}
					
					obsAvd2.state = 0;
				} else if (obsAvd2.frontDistance < MAX_CLEAR_DIST){
					// if the front space is less than the required clearence
					
					if (obsAvd2.currentHeight < liftToHeight && fail == false) {
						// if UAV has not failed (reach the maximum height) continue
						// move up
						obsAvd2.state = 2;
					} else {
						// if you have reached the max height admit that fail
						fail = true;
						
						// turn your heading PI
						goalHeading.angle = obsAvd2.currentHeading - M_PI;
						chHeadSrv.sendGoal(goalHeading);
						
						//wait for the action to return		
						bool finished_before_timeout = chHeadSrv.waitForResult(ros::Duration(30.0));
			
						if (finished_before_timeout) {
							actionlib::SimpleClientGoalState state = chHeadSrv.getState();
							ROS_INFO("Action finished: %s",state.toString().c_str());
						} else {
							ROS_INFO("Action did not finish before the time out.");
						}
						
						// and return to hover height to let the operator continue 
						obsAvd2.lastSwitch = ros::Time::now();
						obsAvd2.lift(HOVER_HEIGHT);
						
						obsAvd2.state = 0;
					}
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
	
	
	
