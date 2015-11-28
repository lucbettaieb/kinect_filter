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
#include <cmath>

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
  // Snapshot boolean guard
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
    if (xyz(0) > furthest_x)
      furthest_x = xyz(0);
  }

  return furthest_x;
}

std::vector<pcl::PointXYZ> Characterizer::getOffsetVec(float offset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<pcl::PointXYZ> point_vec;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Vector3f xyz = cloud->points[i].getVector3fMap();
    pcl::PointXYZ p(xyz(0) - offset, xyz(1), xyz(2));

    point_vec.push_back(p);
  }

  return point_vec;
}

std::vector<float> Characterizer::getErrorVec(std::vector<pcl::PointXYZ> points)
{
  // This helper function assumes that you have already offset or transformed
  // all of the points
  std::vector<float> err_vec;

  for (size_t i = 0; i < points.size(); i++)
  {
    Vector3f xyz = points[i].getVector3fMap();
    err_vec.push_back(xyz(0));
  }

  return err_vec;
}

// http://graphpad.com/support/faq/prisms-algorithm-for-determining-the-automatic-bin-width-when-creating-a-frequency-distribution/

Histogram createHistogram(std::vector<float> errs)
{
  std::vector<float>::const_iterator max, min;
  float max_f, min_f;
  uint errs_size = errs.size();

  max = std::max_element(errs.begin(), errs.end());
  max_f = static_cast<float>(*max);

  min = std::min_element(errs.begin(), errs.end());
  min_f = static_cast<float>(*min);

  float ds_bin_count = 1 + std::log2f(errs_size);
  float ds_bin_width = (max_f - min_f) / ds_bin_count;

  float sum_errs = 0;

  for (size_t i = 0; i < errs_size; i++)
    sum_errs += errs.at(i);

  float bin_width = sum_errs / errs_size;

  Histogram h;

  float range_iter = min_f;
  // Set up bins
  for (size_t i = 0; i < ds_bin_count; i++)
  {
    Bin b;
    b.r_min = range_iter;
    b.r_max = range_iter + bin_width;
    b.quantity = 0;

    if(!(b.r_max > max_f)) // if we have not exceeded expectations
    { // Testting should be done here, not sure if this is needed
      h.push_back(b);
    }
  }

  // Populate bins very inefficiently
  for (size_t i = 0; i < ds_bin_count; i++)  // iterate over bins
  {
    for (size_t j = 0; j < errs_size; j++)  // for each bin iterate over data
    {
      if (errs.at(j) >= h.at(i).r_min && errs.at(j) < h.at(i).r_max)
      {
        h.at(i).quantity++; 
      }
    }
  }

  return h;
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

      //http://www.gnu.org/software/gsl/manual/html_node/Histograms.html
    }

  ros::spinOnce();
  }
}
