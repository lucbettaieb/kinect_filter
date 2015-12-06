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

Characterizer::Characterizer(ros::NodeHandle nh) : pclKinect_clr_ptr_(new pcl::PointCloud<pcl::PointXYZ>)
{
  nh_ = nh;

  pcl_sub_ = nh_.subscribe(g_pcl_topic, 1, &Characterizer::kinectCB, this);
}

Characterizer::~Characterizer()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Characterizer::getCloud()
{
  return pclKinect_clr_ptr_;
}

Histogram Characterizer::getHistogram()
{
  return histogram;
}

void Characterizer::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // Snapshot boolean guard
  if (!g_processing)
  {
    g_got_pcl = true;
    std::cout << "about to try to populate pclKinect_clr_ptr_" << std::endl;
    
    pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr_);

    std::vector<int> indicies;
    pcl::removeNaNFromPointCloud(*pclKinect_clr_ptr_, *pclKinect_clr_ptr_, indicies);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pclKinect_clr_ptr_);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*pclKinect_clr_ptr_);

  }
}

float Characterizer::getFurthest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Don't care about any kind of pointID, just return the furthext X.
  float furthest_x = -1.0;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Vector3f xyz = cloud->points[i].getVector3fMap();
    if (xyz(2) > furthest_x)
      furthest_x = xyz(2);
  }

  return furthest_x;
}

float Characterizer::getClosest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  float closest_z = 9999999;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Vector3f xyz = cloud->points[i].getVector3fMap();
    if (xyz(2) < closest_z)
      closest_z = xyz(2);
  }

  return closest_z;

}

std::vector<pcl::PointXYZ> Characterizer::getOffsetVec(float offset, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::vector<pcl::PointXYZ> point_vec;

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    Vector3f xyz = cloud->points[i].getVector3fMap();
    pcl::PointXYZ p(xyz(0), xyz(1), offset - xyz(2));

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
    
    err_vec.push_back(xyz(2));
  }

  return err_vec;
}

// http://graphpad.com/support/faq/prisms-algorithm-for-determining-the-automatic-bin-width-when-creating-a-frequency-distribution/

void Characterizer::addToHistogram(std::vector<float> errs, float r_min, float r_max)
{
  // Check to see if an appropriate bin is already in the histogram by seeing if r_mid fits between
  // any existing bin.
  bool already_exists_in_histogram = false;
  float r_mid = (r_max + r_min) / 2;

  float accum = 0;
  for (size_t i = 0; i < errs.size(); i++)
      accum += errs.at(i);

  for (size_t i = 0; i < histogram.size(); i++)
  {
    if (r_mid >= histogram.at(i).r_min && r_mid < histogram.at(i).r_max)
    {
      already_exists_in_histogram = true;
      histogram.at(i).quantity++;
      histogram.at(i).err = (histogram.at(i).err + accum/errs.size()/histogram.at(i).quantity) - (histogram.at(i).err / histogram.at(i).quantity);
    }
  }

  // If it doesn't already exist in the histogram, make a new bin and insert it into the correct place.
  if (!already_exists_in_histogram)
  {
    std::cout << "are we gonna die?" << std::endl;
    Bin b;
    b.r_min = r_min;
    b.r_max = r_max;
    
    
    b.err = accum / errs.size();
    b.quantity = 1;

    // Now we have to insert it into the right place

    uint index = 0;
    std::vector<Bin>::iterator pos;
    for (std::vector<Bin>::iterator it = histogram.begin(); it != histogram.end(); ++it)
    {
      if (histogram.at(index).r_max < r_mid && histogram.at(index).r_min >= r_mid)
        pos = it;
      else
        pos = histogram.end();
      
      index++;
    }

    histogram.insert(pos, b);
    std::cout << "we survived this.." << std::endl;
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "character");
  ros::NodeHandle nh;

  if (!nh.getParam("kinect_filter/pointcloud_topic", g_pcl_topic))
    g_pcl_topic = "/camera/depth_registered/points";

  Characterizer c (nh);

  Histogram hist;

  std::vector<float> errors;
  std::vector<pcl::PointXYZ> offsets;
  float far_z, close_z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud;

  while (ros::ok())
  {
    // Check to see if we got a point cloud...
    if (g_got_pcl)
    {
      p_cloud = c.getCloud();
      
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

      far_z = c.getFurthest(p_cloud);
      close_z = c.getClosest(p_cloud);

      std::cout << "far_z: " << far_z << " | close_z: " << close_z << std::endl;

      //avg_z = (far_z + close_z)/2;

      offsets = c.getOffsetVec(far_z, p_cloud);
      errors = c.getErrorVec(offsets);

      c.addToHistogram(errors, close_z, far_z);

      hist = c.getHistogram();
      for (size_t i = 0; i < hist.size(); i++)
      {
        std::cout << "Bin " << i << ": quantity: " << hist.at(i).quantity << "| err: " << hist.at(i).err << " (" << hist.at(i).r_min << ", " << hist.at(i).r_max << ")" << std::endl; 
      }

      ros::Duration(1.0).sleep();
      g_processing = false;
      // give a histogram of distances and mean errors
    }

  ros::spinOnce();
  }
}
