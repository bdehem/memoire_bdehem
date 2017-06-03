/*
 *  This file is part of boris_drone 2016.
 *  For more information, please refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/keyframe.h>
//#include <boris_drone/map/mapping_node.h>

int Keyframe::ID_counter = 0;

Keyframe::Keyframe() {}

Keyframe::Keyframe(const Frame& frame, Camera* cam)
{
  this->camera = cam;
  this->ID = ID_counter++;
  tf::Matrix3x3 drone2world, cam2drone, cam2world;
  double roll, pitch, yaw;
  drone2world.setRPY(frame.pose.rotX, frame.pose.rotY, frame.pose.rotZ);
  cam2drone.setRPY(-PI/2, 0, -PI/2); //TODO use camera for this
  cam2world = drone2world*cam2drone;
  cam2world.getRPY(roll, pitch, yaw);
  this->pose            = frame.pose; //Rotation is to camera, not to drone
  this->pose.rotX       = roll;
  this->pose.rotY       = pitch;
  this->pose.rotZ       = yaw;
  this->img_points      = frame.img_points;
  this->descriptors     = frame.descriptors;
  this->npts            = frame.img_points.size();
  this->n_mapped_pts    = 0;
  this->point_IDs.resize(npts,-1);
  ROS_INFO("Created keyframe %d. It has %d (unmatched) points",ID,npts);
}

Keyframe::Keyframe(const Frame& frame, Camera* cam, const boris_drone::Pose3D pose)
{
  this->camera = cam;
  this->ID = ID_counter++;
  tf::Matrix3x3 drone2world, cam2drone, cam2world;
  double roll, pitch, yaw;
  drone2world.setRPY(pose.rotX, pose.rotY, pose.rotZ);
  cam2drone.setRPY(-PI/2, 0, -PI/2);
  cam2world = drone2world*cam2drone;
  cam2world.getRPY(roll, pitch, yaw);
  this->pose            = pose; //Rotation is to camera, not to drone
  this->pose.rotX       = roll;
  this->pose.rotY       = pitch;
  this->pose.rotZ       = yaw;
  this->img_points      = frame.img_points;
  this->descriptors     = frame.descriptors;
  this->npts            = frame.img_points.size();
  this->n_mapped_pts    = 0;
  this->point_IDs.resize(npts,-1);
  ROS_INFO("Created keyframe %d. It has %d (unmatched) points",ID,npts);
}

Keyframe::~Keyframe()
{
}

void Keyframe::setAsSeeing(int ptID, int idx_in_kf)
{
  std::map<int,int>::iterator it = point_indices.find(ptID);
  if (it==point_indices.end())
  {
    point_IDs[idx_in_kf] = ptID;
    point_indices[ptID] = idx_in_kf;
    n_mapped_pts++;
  }
  else
    ROS_INFO("Trying to put kf %d as seeing pt %d, but it already sees it!",ID,ptID);
}

//Adds to the map of points to keyframes and index within them
void Keyframe::getPointsSeen(std::map<int,std::map<int,int> >& points)
{
  std::pair <int,int> this_observation;
  for (int i = 0; i<npts; i++)
  {
    ROS_DEBUG("keyframe %d at index %d sees point %d",ID,i,point_IDs[i]);
    if (point_IDs[i]>=0)
    {
      this_observation = std::make_pair(ID,i);
      points[point_IDs[i]].insert(this_observation);
    }
  }
}

bool Keyframe::removePoint(int ptID)
{
  std::map<int,int>::iterator it = point_indices.find(ptID);
  if (it==point_indices.end())
  {
    ROS_WARN("Tried to remove point %d from keyframe %d but point is not in keyframe",ptID,ID);
    return false;
  }
  point_IDs[it->second] = -2;
  point_indices.erase(it);
  n_mapped_pts--;
  return (n_mapped_pts<1);
}

void Keyframe::print()
{
  std::cout << "=== Showing Keyframe Info ===" << std::endl << std::endl;
  std::cout << "Keyframe ID : " << ID << std::endl << std::endl;
  std::cout << "Number of keypoints :" << npts << std::endl << std::endl;
  int nmapped = 0;
  for (int i = 0; i < npts; i++)
  {
    nmapped += (point_IDs[i]>=0);
    std::cout << "Point " << i <<":"<< "Index in map = " << point_IDs[i] << std::endl;
  }
  std::cout << "Total points mapped : " << nmapped << std::endl << std::endl;
  std::cout << "=== End Keyframe Info ===" << std::endl << std::endl;


}
