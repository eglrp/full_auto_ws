#include <iostream>
#include <vector>
#include <algorithm>
#include <time.h>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <apriltomav/AprilTagDetections.h>
#include <apriltomav/AprilTagDetection.h>

using namespace std;

geometry_msgs::Pose globalPose;
geometry_msgs::Pose setpoint;
geometry_msgs::Pose man_sp;
bool setpoint_enable;

// Callback for april tag detection
// updates globalPose
void detectionCallback(const boost::shared_ptr<const apriltomav::AprilTagDetections> &msg)
{
	const apriltomav::AprilTagDetections dets = *msg.get();
	const vector<apriltomav::AprilTagDetection> det_list = dets.detections;
	if(det_list.size()){
		const apriltomav::AprilTagDetection det = det_list[0];
		globalPose = det.pose;
		ROS_INFO("Found Apriltag: %f %f %f",det.pose.position.x,det.pose.position.y,det.pose.position.z);
	}
	return;
}

void curPosCallback(const geometry_msgs::PoseStamped cur_position)
{
	//if(!setpoint_enable){
		setpoint = cur_position.pose;
		//ROS_INFO("Got local position updating setpoint");
	//}
	return;
}	

void setPointEnableCallback(const std_msgs::Bool msg){
	if(msg.data)
		ROS_INFO("Enabling Setpoint Mode");
	else
		ROS_INFO("Disabling Setpoint Mode");
	setpoint_enable = msg.data;
	return;
}

void manSPCallback(const geometry_msgs::Pose p){
	ROS_INFO("Received Manual position setpoint.. Updating..");
	man_sp = p;
	return;
}

int main(int argc, char **argv)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  ROS_INFO("April to MAV Translator node started.");

  ros::init(argc, argv, "apriltomav_translator");
  ros::NodeHandle node;
  
  geometry_msgs::PoseStamped p,sp;
  ros::Publisher pose_publisher = node.advertise<geometry_msgs::PoseStamped>("/apriltomav/vision",100);
  //ros::Publisher set_pos_publisher = node.advertise<geometry_msgs::PoseStamped>("/apriltomav/set_position",100);
  ros::Subscriber at_dets_subscriber = node.subscribe<apriltomav::AprilTagDetections>("/apriltags/detections",100,&detectionCallback);
  //ros::Subscriber cur_pos_subscriber = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/local",100,&curPosCallback);
  ros::Subscriber setpoint_enable_sub = node.subscribe<std_msgs::Bool>("/apriltomav/setpoint_enable",100,&setPointEnableCallback);
  ros::Subscriber man_setpoint_sub = node.subscribe<geometry_msgs::Pose>("/apriltomav/man_sp",100,&manSPCallback);

  ros::Rate loop_rate(7);

  int count = 0;
  setpoint_enable = false;
  while (ros::ok())
  {
	  // gettimeofday(&tv,NULL);
    //ROS_INFO("%f %f %f %f %f %f %f\n",globalPose.position.x,globalPose.position.y,globalPose.position.z,
//			globalPose.orientation.z,globalPose.orientation.y,globalPose.orientation.x);
    //ROS_INFO("Sending Pose and count is: %d\n",count);
    p.pose.position.x = globalPose.position.x;
    p.pose.position.y = globalPose.position.y;
    p.pose.position.z = globalPose.position.z;
    p.pose.orientation.x = globalPose.orientation.x;
    p.pose.orientation.y = globalPose.orientation.y;
    p.pose.orientation.z = globalPose.orientation.z;
    p.pose.orientation.w = globalPose.orientation.w;
    p.header.seq = count;
    p.header.frame_id = count;
    p.header.stamp = ros::Time::now();
    //ROS_INFO("Sending Setpoint and count is: %d\n",count);
	/*
    sp.pose.position.x = 0.0;
    sp.pose.position.y = 0.0;
    sp.pose.position.z = 1.5;
    sp.pose.orientation.x = 0.5;  //TODO: set yaw value here
    sp.pose.orientation.y = 0.0;
    sp.pose.orientation.z = 0.0;
    sp.pose.orientation.w = 0.0;
	*/
		
    	if(!setpoint_enable)
		sp.pose = setpoint;
	else
		sp.pose = man_sp;

    	sp.header.seq = count;
    	sp.header.frame_id = count;
    	sp.header.stamp = ros::Time::now();
		
    //set_pos_publisher.publish(sp);
    pose_publisher.publish(p);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  // ros::spin();
  ROS_INFO("April to MAV Translator node stopped.");

  return EXIT_SUCCESS;
}
