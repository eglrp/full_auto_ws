#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <sstream>
#include <fstream>

#include "mavros/CommandBool.h"
#include "ceres_control/DetectionStats.h"

using namespace std;

geometry_msgs::Pose cur_pos;
geometry_msgs::Pose cur_setpoint;
geometry_msgs::Pose man_sp;
bool setpoint_enable;

bool in_offboard = false;	//TODO: this assumes we start in Manual- use the mode service call later

// ROS service call to programmatically enable offboard
// TODO: have made node global because of this - solve elegantly by puttin gin a class later
bool toggleOffboard(bool state){
	ros::NodeHandle node;
	ros::ServiceClient offboard_caller = node.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
	mavros::CommandBool offboard_cmd;
	offboard_cmd.request.value = state;
 	if(offboard_caller.call(offboard_cmd)){
 		ROS_INFO("ROS service call successful!");
 		return true;
 	}
	else{
 		ROS_INFO("ROS service call successful!");
		return false;
	}
	delete node;
}

// ROS service call to programmatically enable LOITER
bool toggleLoiter(bool state){
	ros::NodeHandle node;
	ros::ServiceClient loiter_caller = node.serviceClient<mavros::CommandBool>("/mavros/set_mode");
	mavros::CommandBool loiter_cmd;
	loiter_cmd.request.value = state;
 	if(offboard_caller.call(offboard_cmd)){
 		ROS_INFO("ROS service call successful!");
 		return true;
 	}
	else{
 		ROS_INFO("ROS service call successful!");
		return false;
	}
	delete node;
}

void curPosCallback(const geometry_msgs::PoseStamped cur_position)
{
	cur_pos = cur_position.pose;
	return;
}	

void setPointEnableCallback(const std_msgs::Bool msg){
	if(msg.data)
		ROS_INFO("Enabling Setpoint Mode");
	else
		ROS_INFO("Disabling Setpoint Mode");
	setpoint_enable = msg.data;
	setpoint = cur_pos;		// So that after enabling manual setpoint input, till a manual setpoint is sent we HOLD position
	return;
}

void setSPCallback(const geometry_msgs::Pose p){
	ROS_INFO("Received Manual position setpoint.. Updating..");
	cur_setpoint = p;
	return;
}

//TODO: if this threshold is too high debug using lower values
void detStatsCb(const ceres_control::DetectionStats ds_msg){
	//TODO: what about going back? handle later & what about MISSION mode- how?
	if(ds_msg.det_percentage > 60.0 && !in_offboard){
		ROS_INFO("Detected AprilTag for more than 50 percent time in the last 3 seconds, going to offboard");
		if(toggleOffboard(true)){
			ROS_INFO("Successfully toggled to Offboard");
			in_offboard = true;
		}
	}
	else if(ds_msg.det_percentage < 30.0 && in_offboard){
		ROS_INFO("Not detecting enough AprilTags, going to manual");
		if(toggleOffboard(false)){
			ROS_INFO("Successfully toggled to Manual");
			in_offboard = false;
		}
	}
}

// TODO: refactor with some ROS equivalent to reduce code
float distanceFromTarget(){
	geometry_msgs::Pose p1,p2;
	p1 = cur_pos;
	p2 = cur_setpoint;
	return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) + 
		(p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) + 
			(p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}


int main(int argc, char **argv)
{
	//ROS stuff
  ROS_INFO("Ceres Control node started.");
  ros::init(argc, argv, "ceres_control");
	// TODO: fix this- node is global right now so that offboard service call can be made without additional stack overhead
	ros::NodeHandle node;

  ros::Rate loop_rate(10);	//TODO: find out ideal rate?
  
  //msg & pub sub stuff
  geometry_msgs::PoseStamped p,sp;
  ros::Publisher set_pos_publisher = node.advertise<geometry_msgs::PoseStamped>("/ceres_control/set_position",100);
  ros::Subscriber cur_pos_subscriber = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/local",100,&curPosCallback);
  ros::Subscriber setpoint_enable_sub = node.subscribe<std_msgs::Bool>("/ceres_control/setpoint_enable",100,&setPointEnableCallback);
  ros::Subscriber man_setpoint_sub = node.subscribe<geometry_msgs::Pose>("/ceres_control/set_setpoint",100,&setSPCallback);
  ros::Subscriber at_det_stats_sub = node.subscribe<ceres_control::ATDetectionStats>("/apriltomav/at_det_stats",10,&detStatsCb);
		
  //some init stuff
  int count = 0;
  setpoint_enable = false;
  prev_setpoint.position.x = 0.;
  prev_setpoint.position.y = 0.;
  prev_setpoint.position.z = 0.;
  prev_setpoint.orientation.x = 0.;
  prev_setpoint.orientation.y = 0.;
  prev_setpoint.orientation.z = 0.;
  prev_setpoint.orientation.w = 1.;

  while (ros::ok())
  {
	if(!setpoint_enable){
		float x_diff = sp.pose.position.x - cur_pos.position.x;
		float y_diff = sp.pose.position.y - cur_pos.position.y;
		float z_diff = sp.pose.position.z - cur_pos.position.z;
		
		//validate setpoint sanity	here
		if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
			ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
			sp.pose = prev_setpoint;
			// TODO: shift to manual control?
		}
		else{
			sp.pose = cur_pos;
		}
	}
	else{
		float x_diff = cur_setpoint.position.x - cur_pos.position.x;
		float y_diff = cur_setpoint.position.y - cur_pos.position.y;
		float z_diff = cur_setpoint.position.z - cur_pos.position.z;
		
		//validate setpoint sanity	here
		if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
			ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
			sp.pose = prev_setpoint;
			// TODO: shift to manual control?
		}
		else{
			//if setpoint valid send a setpoint in increments of 100th
			sp.pose.position.x = cur_pos.position.x + x_diff;
			sp.pose.position.y = cur_pos.position.y + y_diff;
			sp.pose.position.z = cur_pos.position.z + z_diff;
			sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
		 	sp.pose.orientation.y = 0;
		 	sp.pose.orientation.z = 0;
		 	sp.pose.orientation.w = 1;
    	}
   }
	sp.header.seq = count;
	sp.header.frame_id = count;
	sp.header.stamp = ros::Time::now();

	set_pos_publisher.publish(sp);
	prev_setpoint = sp.pose;

    ros::spinOnce();
    loop_rate.sleep();
    
    if(!(count % 10)){
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
