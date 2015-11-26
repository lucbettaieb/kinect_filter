/*
 * characterize
 * a ROS node to characterize noise produced by a Microsoft Kinect sensor
 * eecs452 @ cwru f2015
 * 
 * Luc Bettaieb and Jon Koss
 */

#ifndef CHARACTERIZE_H
#define CHARACTERIZE_H

#include <sensor_msgs/PointCloud2.h>
 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

class Characterizer {

public:
  Characterizer(ros::NodeHandle nh);
  ~Characterizer();

public:
  void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void characterizeNoise();
  
  void transformToWall();
  double getError(pcl::PointXYZRGB point);


private:
  ros::NodeHandle nh_;

  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_;

};

#endif  // CHARACTERIZE_H
