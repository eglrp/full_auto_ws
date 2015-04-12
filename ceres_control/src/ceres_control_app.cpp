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
#include "mavros/State.h"
#include "ceres_control/DetectionStats.h"
#include "ceres_control/ChangeState.h"

/*

TODO:
	* Decision based on modes in inner while loop
	* Cascade Mode Strategy
	* FS should be last mode and any strategy can set mode to FS
	* Do we need a switch to manual anymore now that we have radio?
	* Listen to /mavros/state to see if we are in manual
	* Define LocalLoiter to loiter around a safe setpoint

Notes:
	* So during any intermediate stage should we ask Quad to drift?
	* Merge Loiter and Control SP modes as LocalLoiter
	* We can then keep changing the local loiter setpoints
	* And have a safe setpoint as the last known safe local loiter point

Useful Stuff:
	* http://wiki.ros.org/mavros/Plugins
	* http://wiki.ros.org/mavros/FcuModes
	* Use the FCU Modes to control the entire GPS sequence- the mavros change mode 
		allows us to put the quad in AUTO.MISSION and AUTO.LOITER mode
	* Use Plugins for finding out about GPS convergence and for controlling modes

*/

using namespace std;

geometry_msgs::Pose safe_setpoint;
geometry_msgs::Pose cur_position;
geometry_msgs::Pose cur_setpoint;

bool setpoint_enable;
bool in_offboard = false;	//TODO: this assumes we start in Manual- use the mode service call later

geometry_msgs::Pose path[5];
path[0].position.x = 0.0;
path[0].position.y = 0.0;
path[0].position.z = 3.0;

path[1].position.x = 0.0;
path[1].position.y = 1.0;
path[1].position.z = 3.0;

path[2].position.x = 1.0;
path[2].position.y = 1.0;
path[2].position.z = 3.0;

path[3].position.x = 1.0;
path[3].position.y = 0.0;
path[3].position.z = 1.0;

path[4].position.x = 0.0;
path[4].position.y = 0.0;
path[4].position.z = 1.0;

int cur_path_ind = 0;

enum State{
	Manual,				//Quad being controlled manually
	LocalLoiter,	//Quad loitering around a safe constant vision setpoint
	Drift,				//Quad does not have vision setpoints AND does not know what to do
								// Should this ever happen? TODO
	Path,					//Quad following path
	GPS,					//Quad being controlle in GPS
	FS 						//Quad control in Failsafe mode
};
State quad_state = State.LocalLoiter;

// ROS service call to programmatically enable offboard
bool toggleOffboardLoiter(bool state){
	ros::NodeHandle node;
	ros::ServiceClient offboard_caller = node.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
	mavros::CommandBool offboard_cmd;
	offboard_cmd.request.value = state;
 	if(offboard_caller.call(offboard_cmd)){
 		quad_state = LocalLoiter;
 		return true;
 	}
	else{
 		quad_state = Manual;
		return false;
	}
	delete node;
	delete offboard_caller;
	delete offboard_cmd;
}

// TODO: ROS service call to programmatically enable GPS mode
bool changeMode(int mode){
	ros::NodeHandle node;
	ros::ServiceClient gps_caller = node.serviceClient<mavros::CommandBool>("/mavros/set_mode");
	mavros::SetMode gps_mode;
	gps_mode.request.base_mode = mode;
 	if(gps_caller.call(gps_mode)){
 		quad_state = GPS;
 		return true;
 	}
	else{
 		quad_state = LocalLoiter;
		return false;
	}
	delete node;
	delete gps_cmd;
	delete gps_caller;
}

void curPosCallback(const geometry_msgs::PoseStamped cps)
{
	cur_position = cps.pose;
	return;
}	

void switchModeCallback(const ceres_control::ChangeMode msg){
	ROS_INFO("Received Manual setpoint.. Updating..");
	return;
}

void setManualSetpointCallback(const geometry_msgs::Pose p){
	ROS_INFO("Received Manual setpoint.. Updating..");
	cur_setpoint = p;
	quad_state = Control_SP;
	return;
}

//TODO: if this threshold is too high debug using lower values
void detectionStatsCallback(const ceres_control::DetectionStats ds_msg){
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
float distanceFromTarget(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
	return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) + 
		(p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) + 
			(p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}

bool isValid(Pose p){
	//TODO: fill later
}


int main(int argc, char **argv)
{
	//ROS stuff
  ROS_INFO("Ceres Control node started.");
  ros::init(argc, argv, "CeresControllerNode");
	ros::NodeHandle node;

  ros::Rate loop_rate(10);	//TODO: find out ideal rate?
  
  //msg & pub sub stuff
  geometry_msgs::PoseStamped p,sp;

  // It has one job
  ros::Publisher setpoint_publisher = node.advertise<geometry_msgs::PoseStamped>("/ceres_control/send_setpoint",100);

  // Information sources relevent to control
  ros::Subscriber cur_pos_subscriber = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/local",5,&curPosCallback);
  ros::Subscriber cur_state_subscriber = node.subscribe<mavros::State>("/mavros/state",100,&curPosCallback);
  ros::Subscriber setpoint_enable_sub = node.subscribe<std_msgs::Bool>("/ceres_control/setpoint_enable",5,&setPointEnableCallback);
  ros::Subscriber state_sub = node.subscribe<ceres_control::ChangeState>("/ceres_control/change_state",5,&changeStateCallback);
  ros::Subscriber man_setpoint_sub = node.subscribe<geometry_msgs::Pose>("/ceres_control/set_setpoint",5,&setSPCallback);
  ros::Subscriber det_stats_sub = node.subscribe<ceres_control::DetectionStats>("/cerestags/at_det_stats",5,&detectionStatsCb);
		
  //some init stuff
  int count = 0;
  setpoint_enable = false;
  quad_state = State.LocalLoiter;
  safe_setpoint.position.x = 0.;
  safe_setpoint.position.y = 0.;
  safe_setpoint.position.z = 0.;
  safe_setpoint.orientation.x = 0.;
  safe_setpoint.orientation.y = 0.;
  safe_setpoint.orientation.z = 0.;
  safe_setpoint.orientation.w = 3.;

  while (ros::ok())
  {
  	switch(quad_state){

			case LocalLoiter:
			float x_diff = sp.pose.position.x - cur_position.position.x;
			float y_diff = sp.pose.position.y - cur_position.position.y;
			float z_diff = sp.pose.position.z - cur_position.position.z;
			
			//validate setpoint sanity	here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control? FS mode?
			}
			else{
				sp.pose = cur_position;
			}
			break;

/*			case SP_control:
			float x_diff = cur_setpoint.position.x - cur_position.position.x;
			float y_diff = cur_setpoint.position.y - cur_position.position.y;
			float z_diff = cur_setpoint.position.z - cur_position.position.z;
			
			//validate setpoint sanity here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control? FS mode?
			}
			else{
				//if setpoint valid send a setpoint
				// TODO: x_diff can be divided by any multiple to acheive speed
				sp.pose.position.x = cur_position.position.x + x_diff;
				sp.pose.position.y = cur_position.position.y + y_diff;
				sp.pose.position.z = cur_position.position.z + z_diff;
				sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
			 	sp.pose.orientation.y = 0;
			 	sp.pose.orientation.z = 0;
			 	sp.pose.orientation.w = 1;
			}
		  break;
*/			
			case PATH:
			if(distanceFromTarget(cur_setpoint,cur_position) > 0.1){
				cur_setpoint = path[cur_path_ind];
			}
			else{
				ROS_INFO("Moving to next setpoint in path");
				cur_path_ind++;
				cur_setpoint = path[cur_path_ind];
			}

			float x_diff = cur_setpoint.position.x - cur_position.position.x;
			float y_diff = cur_setpoint.position.y - cur_position.position.y;
			float z_diff = cur_setpoint.position.z - cur_position.position.z;
			
			//validate setpoint sanity here
			if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.0){
				ROS_INFO("RED_FLAG: Insane setpoint request, sending current position, please make a sensible setpoint request!");
				sp.pose = safe_setpoint;
				// TODO: shift to manual control? FS mode?
			}
			else{
				//if setpoint valid send a setpoint
				// TODO: x_diff can be divided by any multiple to acheive speed
				sp.pose.position.x = cur_position.position.x + x_diff;
				sp.pose.position.y = cur_position.position.y + y_diff;
				sp.pose.position.z = cur_position.position.z + z_diff;
				sp.pose.orientation.x = 0;	//TODO: set yaw value here as a quaternion change if needed
			 	sp.pose.orientation.y = 0;
			 	sp.pose.orientation.z = 0;
			 	sp.pose.orientation.w = 1;
			}		
			
			break;


		  case GPS:

		  case FS:
			sp.pose = safe_setpoint;
		  break;

		}

		sp.header.seq = count;
		sp.header.frame_id = "local_origin";
		sp.header.stamp = ros::Time::now();

		set_pos_publisher.publish(sp);
		safe_setpoint = sp.pose;

    ros::spinOnce();
    loop_rate.sleep();
    
    if(!(count % 10)){
    	ROS_INFO("Current position is: %f %f %f",cur_position.position.x,cur_position.position.y,cur_position.position.z);
    	ROS_INFO("Current setpoint is: %f %f %f",cur_setpoint.position.x,cur_setpoint.position.y,cur_setpoint.position.z);	
    	ROS_INFO("Current setpoint being sent to MAVROS is: %f %f %f",sp.pose.position.x,sp.pose.position.y,sp.pose.position.z);
    	ROS_INFO("Distance from setpoint is : %f",distanceFromTarget(cur_setpoint,cur_position));
    }
    ++count;
  }

  ROS_INFO("Ceres Controller node stopped.");

  return EXIT_SUCCESS;
}
