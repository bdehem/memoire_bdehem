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

Keyframe::Keyframe(MappingNode* p_global_map)
{
  this->map = p_global_map;
  this->ID  = p_global_map->keyframes.size();
}

Keyframe::Keyframe(MappingNode* p_global_map,boris_drone::Pose3D& pose)
{
  this->map  = p_global_map;
  this->ID   = p_global_map->keyframes.size();
  this->pose = pose;
}

Keyframe::Keyframe(MappingNode* p_global_map,const Frame& frame)
{
  this->ID                  = p_global_map->keyframes.size();
  this->map                 = p_global_map;
  this->unmatched_imgPoints = frame.imgPoints;
}


Keyframe::Keyframe(MappingNode* p_global_map,boris_drone::Pose3D& pose, const Frame& frame)
{
  this->map                 = p_global_map;
  this->ID                  = p_global_map->keyframes.size();
  this->pose                = pose;
  this->unmatched_imgPoints = frame.imgPoints;
  this->descriptors         = frame.descriptors;
  this->matchWithMap(map);
  for (int i=0; i < this->map->keyframes.size()-1; i++)
  {
    this->matchWithKeyframe(*this->map->keyframes[i]);
  }
}

Keyframe::~Keyframe()
{
}

int Keyframe::getID()
{
  return this->ID;
}

boris_drone::Pose3D Keyframe::getPose()
{
  return this->pose;
}

void Keyframe::matchWithMap(MappingNode* map)
{
  if (map->descriptors.rows == 0 || this->descriptors.rows == 0)
  {
    return;
  }
  std::vector<cv::DMatch> simple_matches;
  map->matcher.match(this->descriptors, map->descriptors, simple_matches);

  // threshold test
  std::vector<int> indices_to_remove;
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      int thisIdx = simple_matches[k].queryIdx;
      int mapIdx  = simple_matches[k].trainIdx;
      indices_to_remove.push_back(thisIdx);
      //Add this point to mapped points of the keyframe:
      this->points.push_back(mapIdx);
      this->mapped_imgPoints.push_back(this->unmatched_imgPoints[thisIdx]);
      map->KeyframesSeeingPoint[mapIdx].push_back(this->ID);
    }
  }
  //remove points that have been mapped from unmatched points
  //remove highest indices first, so each index is kept
  std::sort(indices_to_remove.begin(), indices_to_remove.end());
  for (int i = indices_to_remove.size()-1; i>=0; i--)
  {
    this->unmatched_imgPoints.erase(this->unmatched_imgPoints.begin()+indices_to_remove[i]);
  }
}

void Keyframe::matchWithKeyframe(Keyframe other)
{
  if (other.descriptors.rows == 0 || this->descriptors.rows == 0)
  {
    return;
  }
  std::vector< cv::DMatch > simple_matches;
  map->matcher.match(this->descriptors, other.descriptors, simple_matches);

  // threshold test
  std::vector<int> matching_indices_this;
  std::vector<int> matching_indices_other;
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      int thisIdx  = simple_matches[k].queryIdx;
      int otherIdx = simple_matches[k].trainIdx;
      matching_indices_this.push_back(thisIdx);
      matching_indices_other.push_back(otherIdx);

      int newPtIdx = this->map->cloud->size();
      this->points.push_back(newPtIdx);
      other.points.push_back(newPtIdx);
      pcl::PointXYZRGBSIFT point;

      //Coordinates will later be adjusted by BA
      point.x = 0;
      point.y = 0;
      point.z = 0;
      this->map->cloud->points.push_back(point);
      std::vector<int> keyframesSeeingNewPt(2);
      keyframesSeeingNewPt[0] = this->ID;
      keyframesSeeingNewPt[1] = other.ID;
      this->map->KeyframesSeeingPoint.push_back(keyframesSeeingNewPt);
      //TODO idea: do only 1 sba for each pair of keyframes
    }
  }

  //remove points that have been mapped from unmatched points
  //remove highest indices first, so each index is kept
  std::sort(matching_indices_this.begin() , matching_indices_this.end());
  std::sort(matching_indices_other.begin(), matching_indices_other.end());
  for (int i = matching_indices_this.size()-1; i>=0; i--)
  {
    this->unmatched_imgPoints.erase(this->unmatched_imgPoints.begin() + matching_indices_this[i]);
  }
  for (int i = matching_indices_other.size()-1; i>=0; i--)
  {
    other.unmatched_imgPoints.erase(other.unmatched_imgPoints.begin() + matching_indices_other[i]);
  }
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
