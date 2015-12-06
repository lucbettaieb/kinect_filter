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
#include <pcl/filters/filter.h>
 #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/common.hpp>

#include <vector>

struct Bin
{
  float r_min;  // the minimum distance that this bin can account error for
  float r_max;  // the maxiumum distance that this bin can account error for

  float err;  // the average error of points within the distance defined by this bin

  uint quantity;  // how how many points are in the bin
};

// 3x1 column vector of floats
typedef Eigen::Matrix<float, 3, 1> Vector3f;
typedef std::vector<Bin> Histogram;

class Characterizer
{
public:
  Characterizer(ros::NodeHandle nh);
  ~Characterizer();

public:
  void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void characterizeNoise();
  std::vector<pcl::PointXYZ> getOffsetVec(float offset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  std::vector<float> getErrorVec(std::vector<pcl::PointXYZ> points);
  float getFurthest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  float getClosest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void addToHistogram(std::vector<float> errs, float r_min, float r_max);
  // Histogram createHistogram(std::vector<float> errs);


  pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
  Histogram getHistogram();

private:
  ros::NodeHandle nh_;

  Histogram histogram;

  ros::Subscriber pcl_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_clr_ptr_;
};

#endif  // CHARACTERIZE_H
