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

void Keyframe::matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                              std::vector< cv::Point3f >& keyframe_matching_points,
                              std::vector< cv::Point2f >& frame_matching_points)
{
  if (frame.descriptors.rows == 0 || this->descriptors.rows == 0)
  {
    return;
  }
  std::vector< cv::DMatch > simple_matches;
  map->matcher.match(frame.descriptors, this->descriptors, simple_matches);

  // threshold test
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      std::vector< int > v(2);
      v[0] = simple_matches[k].trainIdx;
      v[1] = simple_matches[k].queryIdx;
      idx_matching_points.push_back(v);

      cv::Point3f keyframe_point;
      //TODO attention avant c'était this->cloud (quand cette fonction etait encore utilisee et keypoint avait un cloud)
      pcl::PointXYZRGBSIFT pcl_point = this->map->cloud->points[simple_matches[k].trainIdx];
      keyframe_point.x = pcl_point.x;
      keyframe_point.y = pcl_point.y;
      keyframe_point.z = pcl_point.z;
      keyframe_matching_points.push_back(keyframe_point);
      frame_matching_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
    }
  }
}
