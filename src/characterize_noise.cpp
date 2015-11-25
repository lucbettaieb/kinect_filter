/*
 * characterize
 * a ROS node to characterize noise produced by a Microsoft Kinect sensor
 * eecs452 @ cwru f2015
 * 
 * Luc Bettaieb and Jon Koss
 */

#include <kinect_filter/characterize_noise.h>


Characterizer::Characterizer(ros::NodeHandle nh)
{
	nh_ = nh;
}

Characterizer::~Characterizer()
{
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "character");
	ros::NodeHandle nh;

	Characterizer characterizer(nh);
}
