/*
 * characterize
 * a ROS node to characterize noise produced by a Microsoft Kinect sensor
 * eecs452 @ cwru f2015
 * 
 * Luc Bettaieb and Jon Koss
 */

#include <kinect_filter/characterize_noise.h>
#include <ros/ros.h>
#include <string>

std::string g_pcl_topic;

bool g_got_pcl = false;
bool g_processing = false;

Characterizer::Characterizer(ros::NodeHandle nh)
{
  nh_ = nh;

  pcl_sub_ = nh_.subscribe(g_pcl_topic, 1, &Characterizer::kinectCB, this);
}

Characterizer::~Characterizer()
{
}

void Characterizer::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // TODO(lucbettaieb): May need to add in guard boolean here so only one 'snapshot'
  // is processed at a time.
  if (!g_processing)
  {
    g_got_pcl = true;
    pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr_);
  }
}

float Characterizer::getFurthestX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Don't care about any kind of pointID, just return the furthext X.
  float furthest_x = -1.0;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Vector3f xyz = cloud->points[i].getVector3fMap();
    if (xyz(3) > furthest_x)
      furthest_x = xyz(3);

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "character");
  ros::NodeHandle nh;

  if (!nh.getParam("kinect_filter/pointcloud_topic", g_pcl_topic))
    g_pcl_topic = "/kinect/depth/points";

  Characterizer c (nh);

  while (ros::ok())
  {
    // Check to see if we got a point cloud...
    if (g_got_pcl)
    {
      // Begin processing!
      g_processing = true;

      // Transform the points into a frame that is based at the 
      // lowest plane of the points. TODO(jonathankoss): Does this make sense
      // could also transform the points to the fixed frame of the flat surface
      // that we are viewing

      // Going to assume that the setup is like this:
      //              /-->  [--------wall--------]
      // [o kinect o] --->  [--------wall--------]
      //              \-->  [--------wall--------]

      // OK, so we now have a pcl_Kinect_clr_ptr full of points to deal with.
      // Furthest X point away will be where we transform the points to.

      // Everything is in relation to the kinect_frame

      // ! Instead of doing a literal transform we could just subtract that X from all
      // of the points to find the 'error'.


    }

  ros::spinOnce();
  }
}
