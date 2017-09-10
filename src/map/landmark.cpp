/*
 *  This file is part of ucl_drone 2017.
 *  For more information, please refer
 *  to the corresponding header file.
 *
 *  \author Boris Dehem
 *  \date 2017
 *
 */

#include <ucl_drone/map/landmark.h>

int Landmark::ID_counter = 0;

Landmark::Landmark() {}

Landmark::~Landmark() {}

Landmark::Landmark(cv::Point3d& coordinates, cv::Mat& descriptor)
{
  this->ID = ID_counter++;
  this->coordinates = coordinates;
  this->descriptor  = descriptor;
  this->times_inlier  = 0;
  this->times_outlier = 0;
  this->creation_time = ros::Time::now();
}

void Landmark::updateCoords(cv::Point3d& coordinates)
{
  this->coordinates = coordinates;
}

void Landmark::setAsSeenBy(int kfID)
{
  keyframes_seeing.insert(kfID);
}

bool Landmark::setAsUnseenBy(int kfID)
{
  std::set<int>::iterator it = keyframes_seeing.find(kfID);
  if (it == keyframes_seeing.end())
  {
    ROS_WARN("trying to set landmark %d as not seen by keyframe %d, but it is not set as seeing it",ID,kfID);
    return false;
  }
  keyframes_seeing.erase(it);
  return (keyframes_seeing.size()<2);
}

void Landmark::print()
{
  ROS_INFO("Landmark ID = %d",ID);
  ROS_INFO("  coordinates = %6.4f; %6.4f; %6.4f",coordinates.x,coordinates.y, coordinates.z);
  ROS_INFO("  times_inlier  = %d",times_inlier);
  ROS_INFO("  times_outlier = %d",times_outlier);
  ROS_INFO("  created %f seconds ago", (ros::Time::now() - creation_time).toSec());
  std::cout<<"                                  Seen by Keyframes: ";
  std::set<int>::iterator it;
  for(it = keyframes_seeing.begin();it!=keyframes_seeing.end();++it)
    it==keyframes_seeing.begin() ? std::cout<<*it : std::cout<<", "<<*it;
  std::cout << std::endl;
}
