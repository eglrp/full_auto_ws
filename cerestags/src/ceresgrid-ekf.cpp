#include <iostream>
#include <sstream>
#include <fstream>
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <cerestags/DetectionStats.h>
#include <cerestags/EnableZoh.h>

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
const int HISTORY_WINDOW = 15;

using namespace std;

//global stuff
geometry_msgs::Pose globalPose;
geometry_msgs::PoseWithCovariance globalPoseWC;
AprilTags::TagDetector* m_tagDetector = NULL;
AprilTags::TagCodes m_tagCodes(AprilTags::tagCodes25h9);
AprilTags::TagCodes m_annTag(AprilTags::tagCodes64);
RunningStats m_runningStats[3];
bool m_timing = false;; // print timing information for each tag extraction call
int cur = 0;
bool history[HISTORY_WINDOW];
bool updated;
bool enable_zoh;
double prev_t,cur_t;

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
  double m_tagSize = 0.127; //April tag side length in meters of square black frame
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

  if(detections.size()){
    //stack all ps and Ps and solvePnP
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    double s = 0.105/2.;
    double centre_dist = 0.3;
    int grid_c = 5;
    double x_c,y_c;  // bottom left AT, x y

    AprilTags::TagDetection ann_det;
    bool annDet = false;
    bool gridDet = false;
    for (int i=0; i<detections.size(); i++) {
      
      // This is for the Big Annular April Tag- reject for now
      // TODO: incorp pose from this as well and then fuse
      if(detections[i].id == 0 && m_tagDetector->isUsingAnn){
        ann_det = detections[i];
        annDet = true;
        continue;
      }
      else{
        gridDet = true;
      }

      int num = detections[i].id-1;    //starts with 1 at top left
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
    cv::Matx33f cameraMatrix(
                             m_fx, 0, m_px,
                             0, m_fy, m_py,
                             0,  0,  1);
    cv::Vec4f distParam(0,0,0,0); // all 0?


    double g_x,g_y,g_z;
    double g_yaw, g_pitch, g_roll;
    Eigen::Matrix3d g_wRo;
    if(objPts.size()){

      cv::Mat rvec, tvec;
      cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
      // NOTE: can chose between RANSAC and normal solvepPnP. Atm normal works better
      // cv::solvePnPRansac(objPts, imgPts, cameraMatrix, distParam, rvec, tvec,
      //   false,100,8.0,(0.95 * float(detections.size()) * 4.0));

      cv::Mat r;
      cv::Rodrigues(rvec, r);

      r = r.t();  // rotation of inverse
      cv::Mat new_tvec(-r * tvec); // translation of inverse

      g_wRo <<  r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), 
              r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2),
              r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2);
      wRo_to_euler(g_wRo, g_yaw, g_pitch, g_roll);

      g_x = new_tvec.at<double>(0);
      g_y = new_tvec.at<double>(1);
      g_z = new_tvec.at<double>(2);
      
      //Outlier flagging
      if(  fabs(g_x - m_runningStats[0].Mean()) > 2 * m_runningStats[0].StandardDeviation()
        || fabs(g_y - m_runningStats[1].Mean()) > 2 * m_runningStats[1].StandardDeviation()
        || fabs(g_z - m_runningStats[2].Mean()) > 2 * m_runningStats[2].StandardDeviation())
        ROS_INFO("Flag: %4d %3.4f %3.4f %3.4f Tot: %d",detections.size(),g_x,g_y,g_z,++ignore_cnt);

      m_runningStats[0].Push(g_x);
      m_runningStats[1].Push(g_y);
      m_runningStats[2].Push(g_z);
      ROS_INFO("Grid: %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f",
        g_x,g_y,g_z,g_yaw,g_pitch,g_roll);
    }

    {
      // Estimation from big tag
      double a_x,a_y,a_z;
      double a_yaw,a_pitch,a_roll;
      Eigen::Matrix3d a_wRo;  
      if(annDet){
        objPts.clear();
        imgPts.clear();

        objPts.push_back(cv::Point3f(-0.3, -0.3, 0));
        objPts.push_back(cv::Point3f( 0.3, -0.3, 0));
        objPts.push_back(cv::Point3f( 0.3,  0.3, 0));
        objPts.push_back(cv::Point3f(-0.3,  0.3, 0));

        std::pair<float, float> p1 = ann_det.p[0];
        std::pair<float, float> p2 = ann_det.p[1];
        std::pair<float, float> p3 = ann_det.p[2];
        std::pair<float, float> p4 = ann_det.p[3];
        imgPts.push_back(cv::Point2f(p1.first, p1.second));
        imgPts.push_back(cv::Point2f(p2.first, p2.second));
        imgPts.push_back(cv::Point2f(p3.first, p3.second));
        imgPts.push_back(cv::Point2f(p4.first, p4.second));
        cv::Mat rvec, tvec;
        cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
        cv::Mat r;
        cv::Rodrigues(rvec, r);

        r = r.t();                    // rotation of inverse
        cv::Mat new_tvec(-r * tvec); // translation of inverse

        a_wRo <<  r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), 
                 r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2),
                 r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2);
        wRo_to_euler(a_wRo, a_yaw, a_pitch, a_roll);
        a_x = new_tvec.at<double>(0);
        a_y = new_tvec.at<double>(1);
        a_z = new_tvec.at<double>(2);
        
        //Outlier flagging
        if(  fabs(a_x - m_runningStats[0].Mean()) > 2 * m_runningStats[0].StandardDeviation()
          || fabs(a_y - m_runningStats[1].Mean()) > 2 * m_runningStats[1].StandardDeviation()
          || fabs(a_z - m_runningStats[2].Mean()) > 2 * m_runningStats[2].StandardDeviation())
          ROS_INFO("Flag: %4d %3.4f %3.4f %3.4f Tot: %d",detections.size(),a_x,a_y,a_z,++ignore_cnt);

        m_runningStats[0].Push(a_x);
        m_runningStats[1].Push(a_y);
        m_runningStats[2].Push(a_z);
        ROS_INFO("Annular: %3.4f %3.4f %3.4f %3.4f %3.4f %3.4f",
          a_x,a_y,a_z,a_yaw,a_pitch,a_roll);
      }
    }
    if(gridDet){
      Eigen::Quaterniond q(g_wRo);
      globalPose.position.x = g_x;
      globalPose.position.y = g_y;
      globalPose.position.z = g_z;
      globalPose.orientation.w = 1.0;//q.w();
      globalPose.orientation.x = 0.0;//q.x();
      globalPose.orientation.y = 0.0;//q.y();
      globalPose.orientation.z = 0.0;//q.z();

      globalPoseWC.pose = globalPose;
      globalPoseWC.covariance[0] = 0.0000000001;//m_runningStats[0].Variance();
      globalPoseWC.covariance[7] = 0.0000000001;//m_runningStats[1].Variance();
      globalPoseWC.covariance[14] = 0.0000000001;//m_runningStats[2].Variance();
      globalPoseWC.covariance[21] = 0.0;
      globalPoseWC.covariance[28] = 0.0; 
      globalPoseWC.covariance[35] = 0.0;
      updated = true;
      prev_t = cur_t;
    }
  }
  cur_t = tic();  
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

bool switchZoh(cerestags::EnableZoh::Request &req, cerestags::EnableZoh::Response &res){
  enable_zoh = req.enable;
  if(enable_zoh)
    ROS_INFO("Enabling Zero Order Hold");
  else
    ROS_INFO("Disabling Zero Order Hold");
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  // init apriltags detector
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes,m_annTag);

  // init ROS stuff
  ros::init(argc,argv,"cerestags");
  ros::NodeHandle nh;
  
  // publisher subscribers    
  ROS_INFO("Setting up publisher and subscribers"); 

  image_transport::ImageTransport it(nh);    
  // queue size 1 because we want to work with the latest image always
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  geometry_msgs::PoseStamped p;
  ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/cerestags/vision",1);

  geometry_msgs::PoseWithCovarianceStamped pwc;
  ros::Publisher pwc_publisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/cerestags/visionWC",1);
  memset(&globalPoseWC.covariance,0,sizeof(globalPoseWC.covariance));
  
  cerestags::DetectionStats stats_msg;
  ros::Publisher dets_stats_pub = 
      nh.advertise<cerestags::DetectionStats>("/cerestags/at_det_stats",1);

  ros::ServiceServer zoh_srv = nh.advertiseService("cerestags/zoh",&switchZoh);

  //init stats stuff
  int cnt = 0;
  cur = 0;
  for(int i = 0; i < HISTORY_WINDOW; i++)
    history[i] = false;
  updated = false;

  enable_zoh = false;

  ROS_INFO("Ceres Tags node started.");
  while (ros::ok())
  {
    if(updated || enable_zoh){
      p.pose = globalPose;
      pwc.pose = globalPoseWC;
      p.header.seq = cnt;
      p.header.frame_id = "vision";
      p.header.stamp = ros::Time::now();
      pwc.header.seq = cnt;
      pwc.header.frame_id = "vision";
      pwc.header.stamp = ros::Time::now();
      pwc_publisher.publish(pwc);
      pose_publisher.publish(p);
    }

    //update history
    if(updated)
      history[cur] = true;
    else
      history[cur] = false;
    updated = false;
    cur = (cur + 1) % HISTORY_WINDOW;

    //calculate stats
    int dets = 0;
    for(int i = 0; i < HISTORY_WINDOW; i++){
      if(history[i])
        dets++;
    }
    stats_msg.det_percentage = float(dets)/float(HISTORY_WINDOW) * 100.0;
    stats_msg.det_fps = 0.; //TODO: update fps later
    stats_msg.secs_till_last = cur_t - prev_t;
    dets_stats_pub.publish(stats_msg);

    //NOTE: we continuously spin as fast as we can
    ros::spinOnce();
    cnt++;
  }

  ROS_INFO("CeresTags node stopped.");
  return EXIT_SUCCESS;
}
