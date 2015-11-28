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

// 3x1 column vector of floats
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef std::vector<std::vector<float>> Vector2df;

class Characterizer {

public:
  Characterizer(ros::NodeHandle nh);
  ~Characterizer();

public:
  void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void characterizeNoise();
  
  std::vector<pcl::PointXYZ> getOffsetVec(float offset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::vector<float> getErrorVec(std::vector<pcl::PointXYZ> points);
  float getFurthestX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  Vector2df createHistogram(std::vector<float> err_vec);

private:
  ros::NodeHandle nh_;

  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_clr_ptr_;

};

#endif  // CHARACTERIZE_H
