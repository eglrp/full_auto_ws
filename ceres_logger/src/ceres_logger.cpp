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

#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <mavros/State.h>
#include <cerestags/DetectionStats.h>

using namespace std;

geometry_msgs::cur_cp;
geometry_msgs::cur_ct;
geometry_msgs::cur_sp;
float det_per;


void ctCallback(const geometry_msgs::PoseStamped ct){
  cur_ct = ct.pose;
}

void cpCallback(const geometry_msgs::PoseStamped cp){
  static ros::Time prev_t = ros::Time::now();
  cur_cp = cp.pose;
  if ((ros::Time::now() - prev_t).toSec() < 0.5)
    ROS_INFO("Timeout on current position topic");
  prev_t = ros::Time::now();
}

void spCallback(const geometry_msgs::PoseStamped sp){
  static ros::Time prev_t = ros::Time::now();
  cur_sp = sp.pose;
  if ((ros::Time::now() - prev_t).toSec() < 0.5)
    ROS_INFO("Timeout on current setpoint topic");
  prev_t = ros::Time::now();
}

void dsCallback(const cerestags::DetectionStats msg){
  static ros::Time prev_t = ros::Time::now();
  det_per = msg.det_percentage;
  if ((ros::Time::now() - prev_t).toSec() < 2)
    ROS_INFO("Timeout on Detection Stats topic");
  prev_t = ros::Time::now();
}

int main(int argc, char **argv)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  ROS_INFO("Ceres logger node started.");

  ros::init(argc, argv, "ceres_logger");
  ros::NodeHandle node;
  
  ros::Subscriber cp_sub = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/vision",5,&cpCallback);
  ros::Subscriber sp_sub = node.subscribe<geometry_msgs::PoseStamped>("/ceres_control/set_position",5,&spCallback);
  ros::Subscriber ct_sub = node.subscribe<geometry_msgs::PoseStamped>("/cerestags/vision",5,&ctCallback);
  ros::Subscriber ds_sub = node.subscribe<cerestags::DetectionStats>("/cerestags/det_stats",5,&dsCallback);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();
    // NOTE: time is current time and not time of update of individual data
    ROS_INFO("Time: %f\tCP: %f %f %f\tSP: %f %f %f\tCT: %f %f %f\tDS: %f",
      ros::Time::now(),cur_cp.position.x,cur_cp.position.y,cur_cp.position.z,
        cur_sp.position.x,cur_sp.position.y,cur_sp.position.z,
          cur_ct.position.x,cur_ct.position.y,cur_ct.position.z,det_per);
    loop_rate.sleep();
  }

  ROS_INFO("Ceres logger node stopped.");

  return EXIT_SUCCESS;
}
