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

Keyframe::Keyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Frame& frame, Camera* cam)
{
  this->camera = cam;
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
  this->n_mapped_pts    = 0;
  this->point_IDs.resize(npts,-1);
  ROS_INFO("Created Keyframe %d. It has %d keypoints",ID,npts);
}

Keyframe::Keyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Frame& frame, Camera* cam, const boris_drone::Pose3D pose)
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
  this->cloud           = cloud;
  this->img_points      = frame.img_points;
  this->descriptors     = frame.descriptors;
  this->npts            = frame.img_points.size();
  this->n_mapped_pts    = 0;
  this->point_IDs.resize(npts,-1);
  ROS_INFO("Created Keyframe %d. It has %d keypoints",ID,npts);
}

Keyframe::~Keyframe()
{
}

void match(Keyframe& kf0, Keyframe& kf1, std::vector<cv::Point3d>& points3D,
     std::vector<int>& idx_kf0, std::vector<int>& idx_kf1,
     std::vector<int>& match_ID, std::vector<bool>& point_is_new,
     int& next_point_ID, double threshold)
{
  if (kf0.descriptors.rows == 0 || kf1.descriptors.rows == 0)
    return;
  //thresh was 250
  matchDescriptors(kf0.descriptors, kf1.descriptors, idx_kf0, idx_kf1, threshold);
  int nmatch = idx_kf0.size();
  ROS_INFO("%d matches in matchkeyframes",nmatch);
  ROS_INFO("in match() threshold = %f",threshold);
  int this_ID, pt_ID_kf0, pt_ID_kf1;
  points3D.resize(nmatch);
  match_ID.resize(nmatch);
  point_is_new.resize(nmatch,false);
  int points_in_map = kf0.cloud->points.size();
  for (int i = 0; i<nmatch; i++)
  {
    pt_ID_kf0 = kf0.point_IDs[idx_kf0[i]];
    pt_ID_kf1 = kf1.point_IDs[idx_kf1[i]];
    if ((pt_ID_kf0==-1) && (pt_ID_kf1==-1))
    {
      triangulate(points3D[i], kf0.img_points[idx_kf0[i]], kf1.img_points[idx_kf1[i]], kf0.pose, kf1.pose, kf0.camera, kf1.camera);
      this_ID = next_point_ID++;
      kf0.addPoint(idx_kf0[i], this_ID);
      kf1.addPoint(idx_kf1[i], this_ID);
      match_ID[i]     = this_ID;
      point_is_new[i] = true;
    }
    else if ((pt_ID_kf0==-1)&&(pt_ID_kf1!=-2))
    {
      match_ID[i] = pt_ID_kf1;
      kf0.addPoint(idx_kf0[i],match_ID[i]);
    }
    else if ((pt_ID_kf1==-1)&&(pt_ID_kf1!=-2))
    {
      match_ID[i] = pt_ID_kf0;
      kf1.addPoint(idx_kf1[i],match_ID[i]);
    }
  }
  ROS_INFO("finished matching keyframe %d with keyframe %d. There are %d matching points",kf0.ID, kf1.ID,nmatch);
}

void Keyframe::addPoint(int pt_idx, int pt_ID)
{
  point_idx[pt_ID]  = pt_idx;
  point_IDs[pt_idx] = pt_ID;
  n_mapped_pts++;
}

void Keyframe::removePoint(int ptID)
{
  int index = point_idx[ptID];
  point_IDs[index] = -2;
  n_mapped_pts--;
}

void Keyframe::print()
{
  std::cout << "======== Showing Keyframe Info ========" << std::endl << std::endl;
  std::cout << "Number of keypoints :" << npts << std::endl << std::endl;
  int nmapped = 0;
  for (int i = 0; i < npts; i++)
  {
    nmapped += (point_IDs[i]>=0);
    std::cout << "Point " << i <<":"<< std::endl;
    std::cout << "\t Index in map : " << point_IDs[i] << std::endl;
    if (point_IDs[i]>=0)
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
