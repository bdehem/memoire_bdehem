/*
 *  This file is part of boris_drone 2016.
 *  For more information, please refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/mapping_node.h>

Keyframe::Keyframe(Map* p_global_map, const Frame& frame)
{
  this->map         = p_global_map;
  this->imgPoints   = frame.imgPoints;
  this->descriptors = frame.descriptors;
  this->pose        = frame.pose;
  this->npts        = frame.imgPoints.size();
  //set all pointismapped to false
  this->pointIsMapped.resize(npts,false);
  this->points.resize(npts,-1);


  //Old, to remove when not used anymore:
  this->unmapped_imgPoints  = frame.imgPoints;
}


Keyframe::Keyframe(Map* p_global_map, const boris_drone::Pose3D& pose, const Frame& frame)
{
  this->map         = p_global_map;
  this->imgPoints   = frame.imgPoints;
  this->descriptors = frame.descriptors;
  this->pose        = pose;
  this->npts        = frame.imgPoints.size();
  //set all pointismapped to false
  this->pointIsMapped.resize(npts,false);
  this->points.resize(npts,-1);

  //Old, to remove when not used anymore:
  this->unmapped_imgPoints  = frame.imgPoints;
}

Keyframe::~Keyframe()
{
}

void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2)
{
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> simple_matches;
  matcher.match(descriptors1, descriptors2, simple_matches);
  std::vector<int> indices_of_the_matches;
  double thisdistance, otherdistance;
  size_t idx;
  // threshold test
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    thisdistance = simple_matches[k].distance;
    if (thisdistance < DIST_THRESHOLD)
    {
      //Find if trainIdx has already been matched
      std::vector<int>::iterator it = find(matching_indices_2.begin(),
                         matching_indices_2.end(), simple_matches[k].trainIdx);
      if(it != matching_indices_2.end())
      {
        idx = it - matching_indices_2.begin();
        otherdistance = simple_matches[indices_of_the_matches[idx]].distance;
        //Find if this match is better than the old one, replace it
        if (thisdistance < otherdistance)
        {
          matching_indices_1[idx] = simple_matches[k].queryIdx;
        }
      }
      //If it hasn't been matched yet, add it
      else
      {
        matching_indices_1.push_back(simple_matches[k].queryIdx);
        matching_indices_2.push_back(simple_matches[k].trainIdx);
        indices_of_the_matches.push_back(k); //lol
      }
    }
  }
}


void Keyframe::match(Keyframe& other)
{
  if (this->descriptors.rows == 0 || other.descriptors.rows == 0)
    return;
  std::vector<int> matching_indices_1;
  std::vector<int> matching_indices_2;
  matchDescriptors(this->descriptors, other.descriptors, matching_indices_1, matching_indices_2);
  for (int i = 0; i<matching_indices_1.size();i++)
  {
    ROS_INFO("Matching index %d: kf1 = %d; kf2 = %d",i,matching_indices_1[i],matching_indices_2[i]);
  }


  std::vector<cv::Point3d> points3D;
  int pointsInMap = map->cloud->points.size();
  for (int i = 0; i<matching_indices_1.size();i++)
  {
    if (!pointIsMapped[matching_indices_1[i]] && !other.pointIsMapped[matching_indices_2[i]])
    {
      cv::Point3d pt3d;
      map->triangulate(pt3d, imgPoints[matching_indices_1[i]],
                             other.imgPoints[matching_indices_2[i]],
                             pose, other.pose);
      points3D.push_back(pt3d);
      //add point to map
      pcl::PointXYZRGBSIFT new_point;
      new_point.x = pt3d.x;
      new_point.y = pt3d.y;
      new_point.z = pt3d.z;
      map->cloud->points.push_back(new_point);
      cv::Range row = cv::Range(matching_indices_1[i],matching_indices_1[i]+1);
      map->descriptors.push_back(descriptors(row,cv::Range::all()));

      points[matching_indices_1[i]] = pointsInMap;
      other.points[matching_indices_2[i]] = pointsInMap;
      pointIsMapped[matching_indices_1[i]] = true;
      other.pointIsMapped[matching_indices_2[i]] = true;

      pointsInMap++;
    }
    else if (other.pointIsMapped[matching_indices_2[i]]) //point is mapped on 2
    {
      points[matching_indices_1[i]] = other.points[matching_indices_2[i]];
      pointIsMapped[matching_indices_1[i]] = true;
      int ptIdx = other.points[matching_indices_2[i]];
      cv::Point3d pt3d(map->cloud->points[ptIdx].x,map->cloud->points[ptIdx].y,map->cloud->points[ptIdx].z);
      points3D.push_back(pt3d);
    }
    else // if (pointIsMapped[matching_indices_1[i]]) //point is mapped on 1 but not on 2
    {
      other.points[matching_indices_2[i]] = points[matching_indices_1[i]];
      other.pointIsMapped[matching_indices_2[i]] = true;
      int ptIdx = points[matching_indices_1[i]];
      cv::Point3d pt3d(map->cloud->points[ptIdx].x,map->cloud->points[ptIdx].y,map->cloud->points[ptIdx].z);
      points3D.push_back(pt3d);
    }
  }
  //bundle adjustment
  std::vector<Keyframe*> kfs;
  kfs.push_back(this);
  kfs.push_back(&other);
  map->doBundleAdjustment2(kfs, matching_indices_1, matching_indices_2, true, points3D);
  ROS_INFO("finished matching keyframes. Map now has %lu points",map->cloud->points.size());
  ROS_INFO("descriptor check: %d rows, %d cols", map->descriptors.rows, map->descriptors.cols);
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
      std::cout << "\t x = " << map->cloud->points[points[i]].x << std::endl;
      std::cout << "\t y = " << map->cloud->points[points[i]].y << std::endl;
      std::cout << "\t z = " << map->cloud->points[points[i]].z << std::endl;
    }
  }
  std::cout << "Total points mapped : " << nmapped << std::endl << std::endl;
  std::cout << "========== End Keyframe Info ==========" << std::endl << std::endl;


}
