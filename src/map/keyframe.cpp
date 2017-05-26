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


Keyframe::Keyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                cv::Mat* map_descriptors, const Frame& frame)
{
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
  this->map_descriptors = map_descriptors;
  this->imgPoints       = frame.imgPoints;
  this->descriptors     = frame.descriptors;
  this->npts            = frame.imgPoints.size();
  //set all pointismapped to false
  this->pointIsMapped.resize(npts,false);
  this->points.resize(npts,-1);


  //Old, to remove when not used anymore:
  this->unmapped_imgPoints  = frame.imgPoints;
}

Keyframe::~Keyframe()
{
}

void Keyframe::match(Keyframe& other, std::vector<cv::Point3d>& points3D,
     std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2)
{
  if (this->descriptors.rows == 0 || other.descriptors.rows == 0)
    return;
  matchDescriptors(this->descriptors, other.descriptors, matching_indices_1, matching_indices_2);

  int pointsInMap = this->cloud->points.size();
  int newpoints = 0;
  for (int i = 0; i<matching_indices_1.size();i++)
  {
    if (!pointIsMapped[matching_indices_1[i]] && !other.pointIsMapped[matching_indices_2[i]])
    {
      cv::Point3d pt3d;
      triangulate(pt3d, imgPoints[matching_indices_1[i]],
                        other.imgPoints[matching_indices_2[i]],
                        pose, other.pose);
      points3D.push_back(pt3d);
      //add point to map
      pcl::PointXYZ new_point;
      new_point.x = pt3d.x;
      new_point.y = pt3d.y;
      new_point.z = pt3d.z;
      this->cloud->points.push_back(new_point);
      cv::Range row = cv::Range(matching_indices_1[i],matching_indices_1[i]+1);
      this->map_descriptors->push_back(descriptors(row,cv::Range::all()));

      points[matching_indices_1[i]] = pointsInMap;
      other.points[matching_indices_2[i]] = pointsInMap;
      pointIsMapped[matching_indices_1[i]] = true;
      other.pointIsMapped[matching_indices_2[i]] = true;
      newpoints++;
      pointsInMap++;
    }
    else if (other.pointIsMapped[matching_indices_2[i]]) //point is mapped on 2
    {
      points[matching_indices_1[i]] = other.points[matching_indices_2[i]];
      pointIsMapped[matching_indices_1[i]] = true;
      int ptIdx = other.points[matching_indices_2[i]];
      cv::Point3d pt3d(this->cloud->points[ptIdx].x,this->cloud->points[ptIdx].y,this->cloud->points[ptIdx].z);
      points3D.push_back(pt3d);
    }
    else // if (pointIsMapped[matching_indices_1[i]]) //point is mapped on 1 but not on 2
    {
      other.points[matching_indices_2[i]] = points[matching_indices_1[i]];
      other.pointIsMapped[matching_indices_2[i]] = true;
      int ptIdx = points[matching_indices_1[i]];
      cv::Point3d pt3d(this->cloud->points[ptIdx].x,this->cloud->points[ptIdx].y,this->cloud->points[ptIdx].z);
      points3D.push_back(pt3d);
    }
  }
  ROS_INFO("finished matching keyframes.");
  ROS_INFO("\t %d new points",newpoints);
  ROS_INFO("\t Map now has %lu points",this->cloud->points.size());
  ROS_INFO("\t check: %d = %lu", this->map_descriptors->rows, this->cloud->points.size());
}



void Keyframe::print()
{
  std::cout << "======== Showing Keyframe Info ========" << std::endl << std::endl;
  std::cout << "Number of keypoints :" << npts << std::endl << std::endl;
  int nmapped = 0;
  for (int i = 0; i < npts; i++)
  {
    nmapped += pointIsMapped[i];
    std::cout << "Point " << i <<":"<< std::endl;
    std::cout << "\t Is mapped?     " << pointIsMapped[i] << std::endl;
    std::cout << "\t Index in map : " << points[i] << std::endl;
    if (pointIsMapped[i])
    {
      std::cout << "\t position in map:" << std::endl;
      std::cout << "\t x = " << this->cloud->points[points[i]].x << std::endl;
      std::cout << "\t y = " << this->cloud->points[points[i]].y << std::endl;
      std::cout << "\t z = " << this->cloud->points[points[i]].z << std::endl;
    }
  }
  std::cout << "Total points mapped : " << nmapped << std::endl << std::endl;
  std::cout << "========== End Keyframe Info ==========" << std::endl << std::endl;


}
