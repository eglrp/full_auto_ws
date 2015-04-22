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
#include "mavros/SetMode.h"
#include "ceres_control/DetectionStats.h"
#include "ceres_control/ChangeState.h"
#include "ceres_control/ChangeBehavior.h"
#include "ceres_control/ChangeMode.h"

/*

TODO:
	* Decision based on modes in inner while loop
	* Cascade Mode Strategy
	* FS should be last mode and any strategy can set mode to FS
	* Do we need a switch to manual anymore now that we have radio?
	* Listen to /mavros/state to see if we are in manual

Notes:
	* So during any intermediate stage should we ask Quad to drift?
	* And have a safe setpoint as the last known safe local loiter point

Useful Stuff:
	* http://wiki.ros.org/mavros/Plugins
	* http://wiki.ros.org/mavros/FcuModes
	* Use the FCU Modes to control the entire GPS sequence- the mavros change mode 
		allows us to put the quad in AUTO.MISSION and AUTO.LOITER mode
	* Use Plugins for finding out about GPS convergence and for controlling modes

*/

using namespace std;

geometry_msgs::Pose cur_position;
geometry_msgs::Pose home_position; 
geometry_msgs::Pose safe_setpoint; 
geometry_msgs::Pose center_position;
geometry_msgs::Pose loiterSp;
geometry_msgs::Pose offboardSp;
geometry_msgs::Pose path[5];
int cur_path_ind; 

enum Mode{
	Manual,
	OffboardLoiter,		//Quad loitering around a safe constant vision setpoint
	OffboardSetpoint,	   //Quad has a setpoint to go to
	OffboardMission,	   //Quad following path
	GPS,				      //Quad being controlle in GPS
	FS, 					   //Quad control in Failsafe mode
	Other  					//ALTCTL or POSCTL modes
};
Mode quad_mode;

static const char * ModeStrings[] = { "manual", "off_loiter", "off_sp", "off_mission", "gps", "other" };

const char * getModeString(int enumVal){
  return ModeStrings[enumVal];
}

enum Behavior{
	LocalLoiter,			//Quad loitering around a safe constant vision setpoint
	LocalSetpoint,	   	//Quad has a setpoint to go to
	LocalMission,	   	//Quad following path
	Autonomous
};
Behavior quad_behavior;

static const char * BehaviorStrings[] = { "local_loiter", "local_sp", "local_mission" };

const char * getBehaviorString(int enumVal){
  return BehaviorStrings[enumVal];
}


bool toggleOffboard(bool state){
	ros::NodeHandle node;
	ros::ServiceClient offboard_caller = node.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
	mavros::CommandBool offboard_cmd;
	offboard_cmd.request.value = state;
 	if(offboard_caller.call(offboard_cmd)){
 		quad_mode = OffboardSetpoint;
 		return true;
 	}
	else{
 		quad_mode = Manual;
		return false;
	}
	delete &node;
	delete &offboard_caller;
	delete &offboard_cmd;
}

void curPosCallback(const geometry_msgs::PoseStamped cps){
	cur_position = cps.pose;
	return;
}	

void detectionStatsCallback(const ceres_control::DetectionStats ds_msg){
	if(ds_msg.det_percentage > 60.0 && quad_behavior == Autonomous){
		ROS_INFO("Detected AprilTag for more than 50 percent time in the last 3 seconds, going to offboard");
		if(toggleOffboard(true)){
			ROS_INFO("Successfully toggled to Offboard");
			quad_mode = OffboardSetpoint;
			offboardSp = center_position;
		}
	}
	else if(ds_msg.det_percentage < 30.0 && quad_mode != Manual){
		ROS_INFO("Not detecting enough AprilTags, going to manual");
		if(toggleOffboard(false)){
			ROS_INFO("Successfully toggled to Manual");
			quad_mode = Manual;
		}
	}
}

// TODO: refactor with some ROS equivalent to reduce code
float distanceFromTarget(geometry_msgs::Pose p1,geometry_msgs::Pose p2){
	return sqrt((p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) + 
		(p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) + 
			(p1.position.z-p2.position.z)*(p1.position.z-p2.position.z));
}

void initHome(){
	home_position.position.x = 0.5;
	home_position.position.y = -0.6;
	home_position.position.z = 3.0;
	home_position.orientation.x = 0.;
	home_position.orientation.y = 0.;
	home_position.orientation.z = 0.;
	home_position.orientation.w = 1.;
	return;
}

void init(){
   quad_mode = OffboardLoiter;
   center_position.position.x = 0.5;
   center_position.position.y = -0.6;
   center_position.position.z = 4.0;
   center_position.orientation.x = 0.;
   center_position.orientation.y = 0.;
   center_position.orientation.z = 0.;
   center_position.orientation.w = 1.0;
   initHome();
   safe_setpoint = home_position;
	path[0].position.x = 1.0;
	path[0].position.y = 0;
	path[0].position.z = 3.0;

	path[1].position.x = 1.0;
	path[1].position.y = -1.2;
	path[1].position.z = 3.0;

	path[2].position.x = 0.0;
	path[2].position.y = -1.2;
	path[2].position.z = 3.0;

	path[3].position.x = 0.0;
	path[3].position.y = 0.0;
	path[3].position.z = 3.0;

	path[4].position.x = 0.5;
	path[4].position.y = -0.6;
	path[4].position.z = 3.0;

	cur_path_ind = 0;
}

// THIS IS ALWAYS SUPERIOR TO ANY MODE SET BY COMPUTER
void mavStateCallback(mavros::State msg){
		if (msg.mode == "MANUAL")
			quad_mode = Manual;
		else if(msg.mode == "OFFBOARD"){
			//TODO: do stuff according to behavior
			switch(quad_behavior){ 
			case LocalLoiter:
				quad_mode = OffboardLoiter;
				loiterSp = cur_position;
				break;
			case LocalMission:
				quad_mode = OffboardMission;
				break;
			case LocalSetpoint:
				quad_mode = OffboardSetpoint;
				break;
			}
		}
		else if(msg.mode == "AUTO.MISSION" || msg.mode == "AUTO.LOITER")
			quad_mode = GPS;
		else
			quad_mode = Other;
}

bool changeBehavior(ceres_control::ChangeBehavior::Request &req,
	ceres_control::ChangeBehavior::Response &res){
	if(req.behavior == "loiter"){
		quad_behavior = LocalLoiter;
		loiterSp = cur_position;
		res.success = true;
	}
	else if(req.behavior == "mission"){
		quad_behavior = LocalMission;
		cur_path_ind = 0;
		res.success = true;
	}
	else if(req.behavior == "waypoint"){
		quad_behavior = LocalSetpoint;
		offboardSp.position.x = req.x;
		offboardSp.position.y = req.y;
		offboardSp.position.z = req.z;
		res.success = true;
	}
	else if(req.behavior == "auto"){
		quad_behavior = Autonomous;
		res.success = true;
	}
	else
		res.success = false;	
	return true;
}

bool changeMode(ceres_control::ChangeMode::Request &req,
	ceres_control::ChangeMode::Response &res){
	//TODO: this has somewhat complex behavior, it will first call mavros service
	// to see if the quad mode on Pixhawk changes
	ros::NodeHandle node;
	ros::ServiceClient m_cli = node.serviceClient<mavros::CommandBool>("/mavros/set_mode");
	mavros::SetMode m_srv;
	m_srv.request.base_mode = 0;
	m_srv.request.custom_mode = req.mode;
 	res.success = m_cli.call(m_srv);
	delete &node;
	delete &m_cli;
	delete &m_srv;
	return true;
}

int main(int argc, char **argv)
{
	//ROS stuff
   ROS_INFO("Ceres Control node started.");
   ros::init(argc, argv, "CeresControllerNode");
	ros::NodeHandle node;

	ros::Rate loop_rate(10);	

	geometry_msgs::PoseStamped p,sp;

	// It has one job!
	ros::Publisher setpoint_publisher = node.advertise<geometry_msgs::PoseStamped>("/ceres_control/send_setpoint",5);

	// Information sources relevent to control
	ros::Subscriber cur_pos_subscriber = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/local",5,&curPosCallback);
	ros::Subscriber state_sub = node.subscribe<mavros::State>("/mavros/state",5,&mavStateCallback);
	ros::Subscriber det_stats_sub = node.subscribe<ceres_control::DetectionStats>("/cerestags/at_det_stats",5,&detectionStatsCallback);
		
	ros::ServiceServer behavior_srv = node.advertiseService("ceres_control/behavior",&changeBehavior);
	ros::ServiceServer mode_srv = node.advertiseService("ceres_control/mode",&changeMode);

  	//some init stuff
  	init();
	int count = 0;
	while (ros::ok())
	{
		switch(quad_mode){
			case OffboardLoiter:
			sp.pose.position = loiterSp.position;
			sp.pose.orientation = loiterSp.orientation;
		 	break;

			case OffboardSetpoint:
			sp.pose = offboardSp;
			break;

			case OffboardMission:
			if(distanceFromTarget(cur_position,path[cur_path_ind]) < 0.1){
				ROS_INFO("Moving to next setpoint in path..");
				cur_path_ind++;
			}
			sp.pose = path[cur_path_ind];
			break;

			case Manual:
			sp.pose = center_position;
			break;


			case FS:
			break;

			case GPS:
			// ROS_INFO("Tag4");
			break;
		}

		float x_diff = sp.pose.position.x - cur_position.position.x;
		float y_diff = sp.pose.position.y - cur_position.position.y;
		float z_diff = sp.pose.position.z - cur_position.position.z;
		
		if (abs(x_diff) > 3.0 || abs(y_diff) > 3.0 || abs(z_diff) > 3.5){
			ROS_INFO("RED_FLAG!");
			sp.pose = home_position;
			// TODO: shift to manual control? FS mode?
		}
		sp.header.seq = count;
		sp.header.frame_id = "local_origin";
		sp.header.stamp = ros::Time::now();

		setpoint_publisher.publish(sp);
		safe_setpoint = sp.pose;

   	ros::spinOnce();
   	loop_rate.sleep();
    
		if(!(count % 10)){
			ROS_INFO("OffboardSp: %f %f %f\t",offboardSp.position.x,offboardSp.position.y,offboardSp.position.z);
			ROS_INFO("Quad Mode: %s\tOffboard Behavior: %s",getModeString(quad_mode),getBehaviorString(quad_behavior));
			ROS_INFO("Current position is: %f %f %f",cur_position.position.x,cur_position.position.y,cur_position.position.z);
			ROS_INFO("Current setpoint being sent to MAVROS is: %f %f %f",sp.pose.position.x,sp.pose.position.y,sp.pose.position.z);
			ROS_INFO("Distance from setpoint is : %f",distanceFromTarget(cur_position,sp.pose));
		}
 		++count;
	}

	ROS_INFO("Ceres Controller node stopped.");

	return EXIT_SUCCESS;
}
