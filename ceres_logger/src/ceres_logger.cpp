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

int main(int argc, char **argv)
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  ROS_INFO("Ceres logger node started.");

  ros::init(argc, argv, "cereslogger");
  ros::NodeHandle node;
  
  ros::Subscriber cp_sub = node.subscribe<geometry_msgs::PoseStamped>("/mavros/position/vision",5,&cpCallback);
  ros::Subscriber sp_sub = node.subscribe<geometry_msgs::PoseStamped>("/ceres_control/set_position",5,&spCallback);
  ros::Subscriber ct_sub = node.subscribe<geometry_msgs::PoseStamped>("/cerestags/vision",5,&ctCallback);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    // NOTE: time is current time and not time of update of individual data
    ROS_INFO("Time: %f\tCP: %f %f %f\tSP: %f %f %f\tCT: %f %f %f",
      ros::Time::now(),cur_cp.pose.x,cur_cp.pose.y,cur_cp.pose.z,
        cur_sp.pose.x,cur_sp.pose.y,cur_sp.pose.z,
          cur_ct.pose.x,cur_ct.pose.y,cur_ct.pose.z);
    loop_rate.sleep();
  }

  ROS_INFO("Ceres logger node stopped.");

  return EXIT_SUCCESS;
}
