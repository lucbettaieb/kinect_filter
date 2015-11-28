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
#include <pcl/conversions.h>

struct Bin
{
	float r_min;
	float r_max;

	uint quantity;
};

// 3x1 column vector of floats
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef std::vector<Bin> Histogram;

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

  Histogram createHistogram(std::vector<float> errs);

private:
  ros::NodeHandle nh_;

  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_clr_ptr_;




};

#endif  // CHARACTERIZE_H
