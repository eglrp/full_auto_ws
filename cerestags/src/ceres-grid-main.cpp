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
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag64.h"
#include "AprilTags/RunningStats.h"


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
AprilTags::TagCodes m_tagCodes(AprilTags::tagCodes25h9);
RunningStats m_runningStats[3];
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

  static int ignore_cnt = 0;
  double m_tagSize = 0.127; // April tag side length in meters of square black frame
  double m_fx = 824;
  double m_fy = 826;
  double m_px = 335;
  double m_py = 218;
  
  double t0;
  if (m_timing) {
    t0 = tic();
  }
  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
  if (m_timing) {
    double dt = tic()-t0;
    ROS_INFO("Extracting tags took %f seconds.",dt);
  }

  // ROS_INFO("Detected %d number of detections",detections.size());

  if(detections.size()){
    //stack all ps and Ps and solvePnP
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    double s = m_tagSize/2.;
    double centre_dist = 0.3;
    int grid_c = 5;
    double x_c,y_c;  // bottom left AT, x y
    for (int i=0; i<detections.size(); i++) {
      // NOTE: uncomment this in case you want to reject too close ATs
      // if(detections[i].id % 2)
      //   continue;
      int num = detections[i].id-1;    //because our grid starts with 1 at top left
      int row = num / grid_c;
      int col = num % grid_c;
      x_c = double(col) * centre_dist; 
      y_c = - double(row) * centre_dist; 

      objPts.push_back(cv::Point3f(x_c - s, y_c - s, 0));
      objPts.push_back(cv::Point3f(x_c + s, y_c - s, 0));
      objPts.push_back(cv::Point3f(x_c + s, y_c + s, 0));
      objPts.push_back(cv::Point3f(x_c - s, y_c + s, 0));

      std::pair<float, float> p1 = detections[i].p[0];
      std::pair<float, float> p2 = detections[i].p[1];
      std::pair<float, float> p3 = detections[i].p[2];
      std::pair<float, float> p4 = detections[i].p[3];
      imgPts.push_back(cv::Point2f(p1.first, p1.second));
      imgPts.push_back(cv::Point2f(p2.first, p2.second));
      imgPts.push_back(cv::Point2f(p3.first, p3.second));
      imgPts.push_back(cv::Point2f(p4.first, p4.second)); 
    }

    if(!objPts.size())
      return;

    cv::Mat rvec, tvec;
    cv::Matx33f cameraMatrix(
                             m_fx, 0, m_px,
                             0, m_fy, m_py,
                             0,  0,  1);
    cv::Vec4f distParam(0,0,0,0); // all 0?
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    // NOTE: can chose between RANSAC and normal solvepPnP. Atm normal works better
    // cv::solvePnPRansac(objPts, imgPts, cameraMatrix, distParam, rvec, tvec,
    //   false,100,8.0,(0.95 * float(detections.size()) * 4.0));

    cv::Mat r;
    cv::Rodrigues(rvec, r);

    r = r.t();  // rotation of inverse
    cv::Mat new_tvec(-r * tvec); // translation of inverse

    Eigen::Matrix3d wRo;
    wRo <<  r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), 
            r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2),
            r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2);
    double yaw, pitch, roll;
    double x,y,z;
    wRo_to_euler(wRo, yaw, pitch, roll);
    x = new_tvec.at<double>(0);
    y = new_tvec.at<double>(1);
    z = new_tvec.at<double>(2);

    //Outlier cleaning    
    if(  fabs(x - m_runningStats[0].Mean()) > 2 * m_runningStats[0].StandardDeviation()
      || fabs(y - m_runningStats[1].Mean()) > 2 * m_runningStats[1].StandardDeviation()
      || fabs(z - m_runningStats[2].Mean()) > 2 * m_runningStats[2].StandardDeviation())
    {
      ROS_INFO("Ignore: %4d %3.4f %3.4f %3.4f Tot: %d",detections.size(),x,y,z,++ignore_cnt);
    }

    m_runningStats[0].Push(x);
    m_runningStats[1].Push(y);
    m_runningStats[2].Push(z);
    ROS_INFO("%4d %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f", detections.size(),
      x, y, z,
        yaw,pitch,roll);
    Eigen::Quaterniond q(wRo);
    pose_mutex.lock();
    globalPose.position.x = x;
    globalPose.position.y = y;
    globalPose.position.z = z;
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
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

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
    if(updated){
      pose_mutex.lock();
      p.pose = globalPose;
      pose_mutex.unlock();
      p.header.seq = cnt;
      p.header.frame_id = "vision";
      p.header.stamp = ros::Time::now();
      pose_publisher.publish(p);
    }
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
    // loop_rate.sleep();
    cnt++;
  }

  ROS_INFO("CeresTags node stopped.");
  return EXIT_SUCCESS;
}
