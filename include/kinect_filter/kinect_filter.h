/*
 * kinect_filter
 * a ROS node to remove noise from a microsoft kinect sensor
 * eecs452 @ cwru f2015
 * 
 * Luc Bettaieb and Jon Koss
 */

#ifndef KINECT_FILTER_H
#define KINECT_FILTER_H

#include "kinect_filter/characterize_noise.h"

#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
 #include <Eigen/Eigen>

class KinectFilter
{
public:
  KinectFilter(ros::NodeHandle &nh);
  ~KinectFilter();

  pcl::PointCloud<pcl::PointXYZRGB> remove_error(Histogram noise);
  std::string getPCFrame(){ return pc_frame; }
private:
  ros::NodeHandle nh_;
  std::string pc_frame;
  
  ros::Subscriber sub_in_cloud;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_in_cloud;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_out_cloud;

  bool got_cloud;
  bool processing;


  void in_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);


};


#endif  // KINECT_FILTER_H