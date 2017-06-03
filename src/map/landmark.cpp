
#include <boris_drone/map/landmark.h>

int Landmark::ID_counter = 0;

Landmark::Landmark() {}

Landmark::~Landmark() {}

Landmark::Landmark(cv::Point3d& coordinates, cv::Mat& descriptor)
{
  this->ID = ID_counter++;
  this->coordinates = coordinates;
  this->descriptor  = descriptor;
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
  ROS_INFO("coordinates = %4.2f; %4.2f; %4.2f",coordinates.x,coordinates.y, coordinates.z);
  std::cout<<"seen by:";
  std::set<int>::iterator it;
  for(it = keyframes_seeing.begin();it!=keyframes_seeing.end();++it)
  {
    std::cout << *it << ", ";
  }
  std::cout << std::endl;
}
