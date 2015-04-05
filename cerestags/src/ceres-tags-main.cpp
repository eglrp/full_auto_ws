#include <iostream>
#include <sstream>
#include <fstream>
#include <mutex>
#include <cstring>
#include <cmath>

#include <vector>
#include <list>

#include <algorithm>
#include <time.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <cerestags/DetectionStats.h>

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag64.h"

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

const double TAG_SIZE = 0.12; //in mts
const double ANN_TAG_SIZE = 1.00; //in mts
using namespace std;


//global stuff
std::mutex pose_mutex;
geometry_msgs::Pose globalPose;
AprilTags::TagDetector* m_tagDetector = NULL;
AprilTags::TagCodes m_tagCodes(AprilTags::tagCodes36h11),m_ann_tagCodes(AprilTags::tagCodes64);
bool m_timing = false;; // print timing information for each tag extraction call
int cur = 0;
bool history[30];
bool updated;

double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
  yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
  roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}

void processImage(cv::Mat& image_gray) {
  
  double t0;
  if (m_timing) {
    t0 = tic();
  }
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  if (m_timing) {
    double dt = tic()-t0;
    ROS_INFO("Extracting tags took %f seconds.",dt);
  }

  ROS_INFO("Detected %d number of detections",detections.size());
  if(detections.size()) {
    AprilTags::TagDetection detection = detections[0];
    double m_tagSize = 0.12; // April tag side length in meters of square black frame
    double m_fx = 824; 
    double m_fy = 826;
    double m_px = 335;
    double m_py = 218;
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                           translation, rotation);
    double yaw, pitch, roll;
    wRo_to_euler(rotation, yaw, pitch, roll);
    Eigen::Quaterniond q(rotation);
    ROS_INFO("Position: %f %f %f YPR: %f %f %f",
      translation(0),translation(1),translation(2),yaw,pitch,roll);

    pose_mutex.lock();
    globalPose.position.x = translation(0);
    globalPose.position.y = translation(1);
    globalPose.position.z = translation(2);
    globalPose.orientation.w = 1.0;//q.w();
    globalPose.orientation.x = 0.0;//q.x();
    globalPose.orientation.y = 0.0;//q.y();
    globalPose.orientation.z = 0.0;//q.z();
    pose_mutex.unlock();
    updated = true;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr subscribed_ptr;
  try
  {
    subscribed_ptr = cv_bridge::toCvCopy(msg, "mono8");
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat cur_image_gray = subscribed_ptr->image;
	processImage(cur_image_gray);
}


int main(int argc, char **argv)
{
  // init apriltags detector
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes,m_ann_tagCodes);

  // init ROS stuff
  ros::init(argc,argv,"cerestags");
  ros::NodeHandle nh;
  
  // publisher subscribers    
  ROS_INFO("Setting up publisher and subscribers"); 

  image_transport::ImageTransport it(nh);    
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  geometry_msgs::PoseStamped p;
  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/cerestags/vision",100);
	
  cerestags::DetectionStats stats_msg;
  ros::Publisher dets_stats_pub = 
      nh.advertise<cerestags::DetectionStats>("/cerestags/det_stats",100);

  int cnt = 0;
  //init stats stuff
  for(int i = 0; i < 30; i++)
    history[i] = false;
  cur = 0;
  updated = false;

  ros::Rate loop_rate(10);
  ROS_INFO("Ceres Tags node started.");

  while (ros::ok())
  {
    pose_mutex.lock();
    p.pose.position.x = globalPose.position.x;
    p.pose.position.y = globalPose.position.y;
    p.pose.position.z = globalPose.position.z;
    p.pose.orientation.x = globalPose.orientation.x;
    p.pose.orientation.y = globalPose.orientation.y;
    p.pose.orientation.z = globalPose.orientation.z;
    p.pose.orientation.w = globalPose.orientation.w;
    pose_mutex.unlock();
    p.header.seq = cnt;
    p.header.frame_id = cnt;
    p.header.stamp = ros::Time::now();
    pose_publisher.publish(p);

    //update history
    if(updated)
      history[cur] = true;
    else
      history[cur] = false;
    updated = false;
    cur = (cur + 1) % 30;

    //calculate stats
    int dets = 0;
    for(int i = 0; i < 30; i++){
    if(history[i])
        dets++;
    }
    stats_msg.det_percentage = dets/10.0 * 100;
    // TODO: dummy right now, update later
    stats_msg.det_fps = 0.;
    stats_msg.secs_till_last = 5;

    // NOTE: publishing at 2 Hz
    if(!(cnt % 5))
      dets_stats_pub.publish(stats_msg);

    ros::spinOnce();
    loop_rate.sleep();
    cnt++;
  }

  ROS_INFO("CeresTags node stopped.");
  return EXIT_SUCCESS;
}
