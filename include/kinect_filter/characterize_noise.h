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

typedef Eigen::Matrix<float, 3, 1> Vector3f;

class Characterizer {

public:
  Characterizer(ros::NodeHandle nh);
  ~Characterizer();

public:
  void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void characterizeNoise();
  
  void transformToWall();
  float getError(pcl::PointXYZRGB point);
  float getFurthestX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  ros::NodeHandle nh_;

  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_clr_ptr_;

};

#endif  // CHARACTERIZE_H
