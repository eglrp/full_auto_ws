#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include "mavros/CommandBool.h"
#include "mavros/State.h"
#include "ceres_control/DetectionStats.h"

#include <sstream>
#include <fstream>

using namespace std;

// atomic<geometry_msgs::Pose> cur_pos();
geometry_msgs::Pose cur_pos;
geometry_msgs::Pose cur_setpoint;
geometry_msgs::Pose safe_setpoint;
geometry_msgs::Pose home_position;
geometry_msgs::Pose path[5];

bool loiter_enable;
bool path_enable;
bool offboard_enable;
int cur_path_index;

// TODO: refactor with some ROS equivalent to reduce code
float distanceFromTarget(){
	geometry_msgs::Pose p1,p2;
	p1 = cur_pos;
	p2 = cur_setpoint;
	return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) + 
		(p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) + 
			(p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}

//inits home position and path
void initHome(){
	path[0].position.x = 0.0;
	path[0].position.y = 0.0;
	path[0].position.z = 4.0;

	path[1].position.x = 0.0;
	path[1].position.y = 1.0;
	path[1].position.z = 4.0;

	path[2].position.x = 1.0;
	path[2].position.y = 1.0;
	path[2].position.z = 4.0;

	path[3].position.x = 1.0;
	path[3].position.y = 0.0;
	path[3].position.z = 4.0;

	path[4].position.x = 0.0;
	path[4].position.y = 0.0;
	path[4].position.z = 4.0;

	home_position.position.x = 0.;
	home_position.position.y = 0.;
	home_position.position.z = 4.;
	home_position.orientation.x = 0.;
	home_position.orientation.y = 0.;
	home_position.orientation.z = 0.;
	home_position.orientation.w = 1.;
	return;
}

void resetCallback(const std_msgs::Bool msg){
	if(msg.data){
		ROS_INFO("Resetting Ceres Control.. Will loiter around home postion if in offboard..");
		initHome();
		loiter_enable = true;
		path_enable = false;
		offboard_enable = false;
	}
}

void curPosCallback(const geometry_msgs::PoseStamped cur_position)
{
	cur_pos = cur_position.pose;
	if(path_enable){
		if(distanceFromTarget() < 0.05){
			cur_path_index = (cur_path_index + 1) % 5;
		}
		cur_setpoint = path[cur_path_index];
	}
	return;
}

// ROS service call to programmatically enable offboard
bool enableOffboard(bool state){
	if(offboard_enable == state)
		return true;
	ros::NodeHandle node;
	ros::ServiceClient offboard_caller = node.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
	mavros::CommandBool offboard_cmd;
	offboard_cmd.request.value = state;
 	if(offboard_caller.call(offboard_cmd)){
 		offboard_enable = state;
 		ROS_INFO("Successfully changed mode");
 		return true;
 	}
 	else{
 		ROS_INFO("Cannot change mode");
 		return false;
 	}
	delete &node;
	delete &offboard_caller;
	delete &offboard_cmd;
}

/**
LOITER mode
This should only be called when quad is stable and we are receving 
good vision estimates
*/
void loiterEnableCallback(const std_msgs::Bool msg){
	ROS_INFO("in L Cb");
	if(msg.data){
		ROS_INFO("Enabling LOITER Mode");
		loiter_enable = true;
		cur_setpoint = cur_pos;		//loiter around current position
	}
	else{
		ROS_INFO("Disabling LOITER Mode, setting home position as setpoint");
		loiter_enable = false;
		cur_setpoint = home_position;		//disable loiter, loiter around home
	}
	return;
}

/**
Enable Offboard on Pixhawk and then switch mode to LOITER
*/
void offboardAndLoiterCallback(const std_msgs::Bool msg){
	ROS_INFO("in OL Cb");
	if(msg.data){
		ROS_INFO("Enabling LOITER Mode..");
		if(enableOffboard(true)){
			std_msgs::Bool x;
			x.data = true;
			loiterEnableCallback(x);
		}
		else{
			ROS_INFO("Cannot enable offboard..Try again..");
		}	
	}
	else{
		if(enableOffboard(false))
			ROS_INFO("Disabled offboard..");
		else{
			ROS_INFO("Cannot disable offboard.. DANGER! Exiting Ceres Control.. Use RC now!!");
			exit(0);
		}
		//TODO
	}
}


/**
PATH mode
This should only be called when quad is stable and we are receving 
good vision estimates
*/
void pathEnableCallback(const std_msgs::Bool msg){
	ROS_INFO("in P Cb");
	if(msg.data){
		ROS_INFO("Enabling PATH Mode");
		cur_path_index = 0;
		path_enable = true;
	}
	else{
		ROS_INFO("Disabling PATH Mode, Entering LOITER");
		std_msgs::Bool x;
		x.data = true;
		loiterEnableCallback(x);
		path_enable = false;
	}
	return;
}

void setSPCallback(const geometry_msgs::Pose p){
	ROS_INFO("in LSP Cb");
	if(loiter_enable){
		ROS_INFO("Updating LOITER setpoint.. Updating..");
		cur_setpoint = p;
	}
	else{
		ROS_INFO("REJECT: This only works in LOITER mode");
	}
	return;
}

void detectionStatsCallback(){
	//TODO
}

string cur_mode = "MANUAL";
bool off_loiter;
void mavStateCallback(mavros::State msg){
	if(msg.mode == "AUTO.MISSION"){
		//do something
		ROS_INFO("Mode: AUTO.MISSION");
	}
	else if (msg.mode == "MANUAL"){
		if(off_loiter){
			off_loiter = false;
			ROS_INFO("Mode: MANUAL.. disabling loiter");
		}
	}
	else if(msg.mode == "OFFBOARD"){
		if(!off_loiter){
			ROS_INFO("Mode: OFFBOARD.. enabling loiter");
			std_msgs::Bool x;
			x.data = true;
			loiterEnableCallback(x);			
			off_loiter = true;
		}
	}
}

int main(int argc, char **argv)
{
	off_loiter = false;
  ros::init(argc, argv, "ceres_control");
  ros::NodeHandle node;
  
  ROS_INFO("Ceres Control node started.");

  geometry_msgs::PoseStamped p,sp;

  ros::Publisher set_pos_publisher = node.advertise<geometry_msgs::PoseStamped>("/ceres_control/set_position",5);

  //cur pos sub
  ros::Subscriber cur_pos_subscriber = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/local",5,&curPosCallback);
  //loiter setpoint sub
  ros::Subscriber man_setpoint_sub = node.subscribe<geometry_msgs::Pose>("/ceres_control/set_setpoint",5,&setSPCallback);
  //detection stats sub
  // ros::Subscriber det_stats_sub = node.subscribe<ceres_control::DetectionStats>("/cerestags/det_stats",5,&detectionStatsCallback);
	//mode chage subs  
  ros::Subscriber path_enable_sub = node.subscribe<std_msgs::Bool>("/ceres_control/path_enable",5,&pathEnableCallback);
  ros::Subscriber loiter_enable_sub = node.subscribe<std_msgs::Bool>("/ceres_control/loiter_enable",5,&loiterEnableCallback);
  ros::Subscriber offloiter_enable_sub = node.subscribe<std_msgs::Bool>("/ceres_control/offboard_loiter_enable",5,&offboardAndLoiterCallback);
  ros::Subscriber reset_sub = node.subscribe<std_msgs::Bool>("/ceres_control/reset",5,&resetCallback);
  ros::Subscriber state_sub = node.subscribe<mavros::State>("/mavros/state",5,&mavStateCallback);

  ros::Rate loop_rate(10);

  //initialize
  int count = 0;
  loiter_enable = false;
  path_enable = false;
  cur_path_index = 0;
  safe_setpoint = home_position;

  //TODO: streamline this with the knowledge of threading
  while (ros::ok())
  {
		if(!loiter_enable && !path_enable){
			float x_diff = sp.pose.position.x - cur_pos.position.x;
			float y_diff = sp.pose.position.y - cur_pos.position.y;
			float z_diff = sp.pose.position.z - cur_pos.position.z;
			
			//validate setpoint sanity here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control?
			}
			else{
				//drift
				sp.pose = cur_pos;	// TODO: drift or home?
			}
		}
		else if (loiter_enable){
			float x_diff = cur_setpoint.position.x - cur_pos.position.x;
			float y_diff = cur_setpoint.position.y - cur_pos.position.y;
			float z_diff = cur_setpoint.position.z - cur_pos.position.z;
			
			//validate setpoint sanity	here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending safe_setpoint, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control?
			}
			else{
				sp.pose.position.x = cur_pos.position.x + x_diff;
				sp.pose.position.y = cur_pos.position.y + y_diff;
				sp.pose.position.z = cur_pos.position.z + z_diff;
				sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
			 	sp.pose.orientation.y = 0;
			 	sp.pose.orientation.z = 0;
			 	sp.pose.orientation.w = 1;
	    }
	  }
	  else if (path_enable){
			float x_diff = cur_setpoint.position.x - cur_pos.position.x;
			float y_diff = cur_setpoint.position.y - cur_pos.position.y;
			float z_diff = cur_setpoint.position.z - cur_pos.position.z;
			
			//validate setpoint sanity	here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending safe_setpoint, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control?
			}
			else{
				//if setpoint valid send a setpoint in increments of 10th
				sp.pose.position.x = cur_pos.position.x + x_diff / 10;
				sp.pose.position.y = cur_pos.position.y + y_diff / 10;
				sp.pose.position.z = cur_pos.position.z + z_diff / 10;
				sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
			 	sp.pose.orientation.y = 0;
			 	sp.pose.orientation.z = 0;
			 	sp.pose.orientation.w = 1;
	    }
	  }
		sp.header.seq = count;
		sp.header.frame_id = count;
		sp.header.stamp = ros::Time::now();
		
		sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
	 	sp.pose.orientation.y = 0;
	 	sp.pose.orientation.z = 0;
	 	sp.pose.orientation.w = 1;
		set_pos_publisher.publish(sp);
		safe_setpoint = sp.pose;

    ros::spinOnce();
    loop_rate.sleep();
    
    if(!(count % 20)){
    	ROS_INFO("Current position is: %f %f %f",cur_pos.position.x,cur_pos.position.y,cur_pos.position.z);
    	ROS_INFO("Current setpoint is: %f %f %f",cur_setpoint.position.x,cur_setpoint.position.y,cur_setpoint.position.z);	
    	ROS_INFO("Current setpoint being sent to MAVROS is: %f %f %f",sp.pose.position.x,sp.pose.position.y,sp.pose.position.z);
    	ROS_INFO("Distance from setpoint is : %f",distanceFromTarget());
    }
    ++count;
  }

  ROS_INFO("Ceres Controller node stopped.");

  return EXIT_SUCCESS;
}
