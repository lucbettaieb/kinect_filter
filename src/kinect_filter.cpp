/*
 * kinect_filter
 * a ROS node to remove noise from a microsoft kinect sensor
 * eecs452 @ cwru f2015
 * 
 * Luc Bettaieb and Jon Koss
 */

#include "kinect_filter/kinect_filter.h"


KinectFilter::KinectFilter(ros::NodeHandle &nh) :
  p_in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  nh_ = nh;

  sub_in_cloud = nh_.subscribe("camera/depth_registered/points", 1, &KinectFilter::in_cloud_cb, this);

  got_cloud = false;
  processing = false;
}

KinectFilter::~KinectFilter()
{
}

void KinectFilter::in_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  if (!processing)
  {
    pcl::fromROSMsg(*cloud, *p_in_cloud);
    got_cloud = true;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectFilter::remove_error(Histogram noise)
{
  // Only remove Z error for now....

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_out_cloud_ret;

  for (size_t i = 0; i < p_in_cloud->size(); i++)
  {
    Eigen::Vector3f xyz = p_in_cloud->points.at(i).getVector3fMap();
    Eigen::Vector3i rgb;
    rgb = p_in_cloud->points.at(i).getRGBVector3i();

    for (size_t j = 0; j < noise.size(); j++)
    {
      if (xyz(2) >= noise.at(j).r_min && xyz(2) < noise.at(j).r_max)
      {   
        xyz(2) -= noise.at(j).err;

        pcl::PointXYZRGB filtered_p(rgb(0), rgb(1), rgb(2));
        filtered_p.getVector3fMap() = xyz;

        p_out_cloud_ret->points.push_back(filtered_p);

        break;
      }
    }
  }

  return p_out_cloud_ret;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_filter");
  ros::NodeHandle nh;

  Histogram hist;

  KinectFilter kf(nh);

  ros::Publisher pub_out_cloud = nh.advertise<sensor_msgs::PointCloud2>("camera/depth_registered/filtered/points", 1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_out_cloud;

  std::fstream myfile("/home/luc/indigo/histogram.dat", std::ios_base::in);
  


  float a;
  int i = 0;
  int j;
  while (myfile >> a)
  {
    j = std::floor(i/4); 
    Bin b;
    if(i%4 == 0)
    {
      b.quantity = a; 
    } else if(i%4 == 1)
    {
      b.err = a;
    } else if(i%4 == 2)
    {
      b.r_min = a;
    } else if(i%4 == 3)
    {
      b.r_max = a;
      hist.push_back(b);
    }
    
    i++;
  }
  sensor_msgs::PointCloud2 ros_view_cloud;

  ROS_INFO("got histogram...");
  for (size_t i = 0; i < hist.size(); i++)
  {
    std::cout << "Bin " << i << ": quantity: " << hist.at(i).quantity << "| err: " << hist.at(i).err << " (" << hist.at(i).r_min << ", " << hist.at(i).r_max << ")" << std::endl; 
  }

  ROS_INFO("about to filter...");
  while(ros::ok)
  {
    p_out_cloud = kf.remove_error(hist);

    pcl::toROSMsg(*p_out_cloud, ros_view_cloud);

    pub_out_cloud.publish(ros_view_cloud);

    ros::spinOnce();
  }
  

}