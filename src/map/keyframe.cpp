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

Keyframe::Keyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Frame& frame)
{
  this->ID = ID_counter++;
  tf::Matrix3x3 drone2world, cam2drone, cam2world;
  double roll, pitch, yaw;
  drone2world.setRPY(frame.pose.rotX, frame.pose.rotY, frame.pose.rotZ);
  cam2drone.setRPY(-PI/2, 0, -PI/2);
  cam2world = drone2world*cam2drone;
  cam2world.getRPY(roll, pitch, yaw);
  this->pose            = frame.pose; //Rotation is to camera, not to drone
  this->pose.rotX       = roll;
  this->pose.rotY       = pitch;
  this->pose.rotZ       = yaw;
  this->cloud           = cloud;
  this->img_points      = frame.img_points;
  this->descriptors     = frame.descriptors;
  this->npts            = frame.img_points.size();
  //set all point_is_mapped to false
  this->point_is_mapped.resize(npts,false);
  this->point_IDs.resize(npts,-1);
  ROS_INFO("Created new Keyframe. ID is %d",ID);
}

Keyframe::~Keyframe()
{
}

void match(Keyframe& kf0, Keyframe& kf1, std::vector<cv::Point3d>& points3D,
     std::vector<int>& idx_kf0, std::vector<int>& idx_kf1,
     std::vector<int>& match_ID, std::vector<bool>& point_is_new,
     int& next_point_ID)
{
  if (kf0.descriptors.rows == 0 || kf1.descriptors.rows == 0)
    return;
  matchDescriptors(kf0.descriptors, kf1.descriptors, idx_kf0, idx_kf1);
  int nmatch = idx_kf0.size();
  int this_ID;
  points3D.resize(nmatch);
  match_ID.resize(nmatch);
  point_is_new.resize(nmatch,false);
  int points_in_map = kf0.cloud->points.size();
  for (int i = 0; i<nmatch; i++)
  {
    if (!kf0.point_is_mapped[idx_kf0[i]] && !kf1.point_is_mapped[idx_kf1[i]])
    {
      triangulate(points3D[i], kf0.img_points[idx_kf0[i]], kf1.img_points[idx_kf1[i]], kf0.pose, kf1.pose);
      this_ID = next_point_ID++;
      kf0.addPoint(idx_kf0[i], this_ID);
      kf1.addPoint(idx_kf1[i], this_ID);
      match_ID[i]     = this_ID;
      point_is_new[i] = true;
    }
    else if (!kf0.point_is_mapped[idx_kf0[i]])
    {
      kf0.addPoint(idx_kf0[i], kf1.point_IDs[idx_kf1[i]]);
      match_ID[i]            = kf1.point_IDs[idx_kf1[i]];
    }
    else if (!kf1.point_is_mapped[idx_kf1[i]])
    {
      kf1.addPoint(idx_kf1[i], kf0.point_IDs[idx_kf0[i]]);
      match_ID[i]            = kf0.point_IDs[idx_kf0[i]];
    }
  }
  ROS_INFO("finished matching keyframe %d with keyframe %d.",kf0.ID, kf1.ID);
}

void Keyframe::addPoint(int pt_idx, int pt_ID)
{
  point_idx[pt_ID]  = pt_idx;
  point_IDs[pt_idx] = pt_ID;
  point_is_mapped[pt_idx] = true;
}

void Keyframe::removePoint(int ptID)
{
  int index = point_idx[ptID];
  point_is_mapped[index] = false;
  point_IDs[index] = -2;
}

void Keyframe::print()
{
  std::cout << "======== Showing Keyframe Info ========" << std::endl << std::endl;
  std::cout << "Number of keypoints :" << npts << std::endl << std::endl;
  int nmapped = 0;
  for (int i = 0; i < npts; i++)
  {
    nmapped += point_is_mapped[i];
    std::cout << "Point " << i <<":"<< std::endl;
    std::cout << "\t Is mapped?     " << point_is_mapped[i] << std::endl;
    std::cout << "\t Index in map : " << point_IDs[i] << std::endl;
    if (point_is_mapped[i])
    {
      std::cout << "\t position in map:" << std::endl;
      std::cout << "\t x = " << this->cloud->points[point_IDs[i]].x << std::endl;
      std::cout << "\t y = " << this->cloud->points[point_IDs[i]].y << std::endl;
      std::cout << "\t z = " << this->cloud->points[point_IDs[i]].z << std::endl;
    }
  }
  std::cout << "Total points mapped : " << nmapped << std::endl << std::endl;
  std::cout << "========== End Keyframe Info ==========" << std::endl << std::endl;


}
