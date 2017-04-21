

/*!
* This method searches among previously mapped Keyframes the closest one with the given Frame
* @param[in]  pose          The estimated pose
* @param[out] keyframe_ID   The identification number of the closest Keyframe
* @param[in]  current_frame The frame object
*/
bool closestKeyframe(const boris_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame);



  /*!
   * This method searches all keyframes containing keypoints visible frome the pose given
   * @param[in]  pose         The pose of the drone
   * @param[out] keyframes_ID Identification number of all keyframes in the map visible from the given pose
   */
  void getVisibleKeyframes(const boris_drone::Pose3D& pose,
                           std::vector<std::vector<int> >& keyframes_ID);

  /*! This method searches all keypoints visible frome the pose given
   * \param[in]  pose The pose of the drone
   * \param[out] idx  Indexes of all keypoints in the map visible from the given pose
   */
  void getVisiblePoints(const boris_drone::Pose3D& pose, std::vector<int>& idx);











bool Map::closestKeyframe(const boris_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame)
{
  std::vector< std::vector< int > > keyframes_ID;
  this->getVisibleKeyframes(pose, keyframes_ID);

  int best_keyframe = -1;
  int best_keyframe_inlier_size = 0;
  std::vector<int> inliers;
  boris_drone::Pose3D PnP_pose;
  Keyframe* reference_keyframe_candidate;

  for (int i = 0; i < keyframes_ID.size(); i++)
  {
    // check if enough points for PnP
    if (keyframes_ID[i][1] > threshold_lost)
    {
      // ROS_DEBUG("keyframe #%d: enough points for PnP (i=%d)", keyframes_ID[i][0], i);
      // PnP number of inliers test
      reference_keyframe_candidate = this->keyframes[keyframes_ID[i][0]];
      bool PnP_success = this->doPnP(current_frame, PnP_pose, inliers, reference_keyframe_candidate);

      // filter out bad PnP estimation not yet treated: case where PnP thinks he has a good pose
      // estimation while he actually has found the bad clone configuration (symmetry in ground
      // plane:
      // like a mirror effect), resulting in negative altitude and bad (uninterpretable?) other
      // meausures
      if (abs(PnP_pose.z - current_frame.pose.z) > 0.5)  // treshold in m
      {
        PnP_success = false;
      }

      if (PnP_success)
      {
        // PnP number of inliers test
        if (inliers.size() > best_keyframe_inlier_size &&
            !this->newKeyframeNeeded(inliers.size(), reference_keyframe_candidate))
        {
          best_keyframe = keyframes_ID[i][0];
          best_keyframe_inlier_size = inliers.size();
        }
        if (inliers.size() > 1.1 * threshold_new_keyframe)
        {
          break;
        }
      }
    }
    else
    {
      break;  // because keyframes_ID is sorted in decreasing order with respect to number of visible points
    }
  }

  if (best_keyframe == -1)
  {
    return false;
  }
  keyframe_ID = best_keyframe;
  return true;
}

void Map::getVisibleKeyframes(const boris_drone::Pose3D& pose, std::vector<std::vector<int> >& keyframes_ID)
{
  std::vector<int> idx;
  this->getVisiblePoints(pose, idx);
  int j;
  for (unsigned i = 0; i < idx.size(); ++i)
  {
    std::vector<int> ids = this->KeyframesSeeingPoint[idx[i]];
    for (int j; j < ids.size();j++)
    {
      int k;
      for (k = 0; k < keyframes_ID.size(); k++)
      {
        if (ids[j] == keyframes_ID[k][0])
        {
          keyframes_ID[k][1]++;
          break;
        }
      }
      if (k == keyframes_ID.size())
      {
        std::vector<int> vec(2);
        vec[0] = ids[j];
        vec[1] = 1;
        keyframes_ID.push_back(vec);
      }
    }
  }

  std::sort(keyframes_ID.begin(), keyframes_ID.end(), customLess);
}

void Map::getVisiblePoints(const boris_drone::Pose3D& pose, std::vector<int>& idx)
{
  double yaw = -pose.rotZ;
  double pitch = -pose.rotY;
  double roll = -pose.rotX;

  cv::Mat world2drone = rollPitchYawToRotationMatrix(roll, pitch, yaw);
  // cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
  cv::Mat_< double > drone2cam = (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
  cv::Mat world2cam = drone2cam * world2drone;

  cv::Mat world_plane_top = world2cam.t() * cam_plane_top;
  cv::Mat world_plane_right = world2cam.t() * cam_plane_right;
  cv::Mat world_plane_bottom = world2cam.t() * cam_plane_bottom;
  cv::Mat world_plane_left = world2cam.t() * cam_plane_left;
  cv::Mat translation = (cv::Mat_< double >(3, 1) << pose.x, pose.y, pose.z);
  double d_top = -translation.dot(world_plane_top);
  double d_right = -translation.dot(world_plane_right);
  double d_bottom = -translation.dot(world_plane_bottom);
  double d_left = -translation.dot(world_plane_left);

  for (unsigned i = 0; i < cloud->points.size(); ++i)
  {
    cv::Mat point = (cv::Mat_< double >(3, 1) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    double t_top = point.dot(world_plane_top) + d_top;
    double t_right = point.dot(world_plane_right) + d_right;
    double t_bottom = point.dot(world_plane_bottom) + d_bottom;
    double t_left = point.dot(world_plane_left) + d_left;

    if (t_top >= 0 && t_right >= 0 && t_bottom >= 0 && t_left >= 0)
    {
      idx.push_back(i);
    }
  }
}
