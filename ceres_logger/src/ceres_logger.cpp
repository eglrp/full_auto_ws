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

/*
TODO:
  print out logging info
  have timeouts on nodes to alert if one is not working
 

*/

geometry_msgs::cur_cp;
geometry_msgs::cur_ct;
geometry_msgs::cur_sp;


void ctCallback(const geometry_msgs::PoseStamped ct){
  cur_ct = ct.pose;
}

void cpCallback(const geometry_msgs::PoseStamped cp){
  cur_cp = cp.pose;
}

void spCallback(const geometry_msgs::PoseStamped sp){
  cur_sp = sp.pose;
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

  ros::Rate loop_rate(15);

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Time: %f\tCP: %f %f %f\tSP: %f %f %f\tCT: %f %f %f",
      ros::Time::now(),)
    loop_rate.sleep();
  }

  ROS_INFO("April to MAV Translator node stopped.");

  return EXIT_SUCCESS;
}
