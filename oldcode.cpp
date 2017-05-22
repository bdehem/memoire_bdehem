

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




bool Map::triangulate1(cv::Point3d& pt_out, const cv::Point2d& pt1, const cv::Point2d& pt2,
                      const boris_drone::Pose3D& pose1, const boris_drone::Pose3D& pose2)
{
  //Midpoint method
  //http://geomalgorithms.com/a07-_distance.html
  double angleThresh = PI/40;

  /* get coordinates */
  cv::Mat cam2world1, cam2world2, origin1, origin2;
  getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  getCameraPositionMatrices(pose2, cam2world1, origin2, true);

  /* compute point coordinates in calibrated image coordinates (=rays in camera coordinates) */
  cv::Mat ray1_cam = (cv::Mat_<double>(3, 1) << (pt1.x - Read::img_center_x()) / Read::focal_length_x(),
                                                (pt1.y - Read::img_center_y()) / Read::focal_length_y(), 1);
  cv::Mat ray2_cam = (cv::Mat_<double>(3, 1) << (pt2.x - Read::img_center_x()) / Read::focal_length_x(),
                                                (pt2.y - Read::img_center_y()) / Read::focal_length_y(), 1);
  /* convert to world coordinates */
  cv::Mat ray1 = cam2world1 * ray1_cam;
  cv::Mat ray2 = cam2world1 * ray2_cam;

  ROS_INFO("\tray1_cam=%f,%f,%f",ray1_cam.at<double>(0,0), ray1_cam.at<double>(1,0), ray1_cam.at<double>(2,0));
  ROS_INFO("\tray2_cam=%f,%f,%f",ray2_cam.at<double>(0,0), ray2_cam.at<double>(1,0), ray2_cam.at<double>(2,0));
  ROS_INFO("\tray1=%f,%f,%f",ray1.at<double>(0,0), ray1.at<double>(1,0), ray1.at<double>(2,0));
  ROS_INFO("\tray2=%f,%f,%f",ray2.at<double>(0,0), ray2.at<double>(1,0), ray2.at<double>(2,0));

  //if (angleBetween(ray1,ray2) < angleThresh)
  //  return false;

  /* compute closest points on both rays and take midpoint*/
  double a,b,c,d,e,k1,k2,denominator;
  cv::Mat p1, p2, temp;
  temp = ray1.t() * ray1; a = temp.at<double>(0, 0);
  temp = ray1.t() * ray2; b = temp.at<double>(0, 0);
  temp = ray2.t() * ray2; c = temp.at<double>(0, 0);
  temp = ray1.t() * (origin1 - origin2); d = temp.at<double>(0, 0);
  temp = ray2.t() * (origin1 - origin2); e = temp.at<double>(0, 0);
  denominator = (a*c) - (b*b);
  k1 = ((b*e) - (c*d))/denominator;
  k2 = ((a*e) - (b*d))/denominator;
  p1 = origin1 + k1*ray1;
  p2 = origin2 + k2*ray2;

  pt_out.x = (p1.at<double>(0, 0) + p2.at<double>(0,0))/2;
  pt_out.y = (p1.at<double>(1, 0) + p2.at<double>(1,0))/2;
  pt_out.z = (p1.at<double>(2, 0) + p2.at<double>(2,0))/2;
  return true;
}

bool Map::triangulate2(cv::Point3d& pt_out, const cv::Point2d& pt1, const cv::Point2d& pt2,
                      const boris_drone::Pose3D& pose1, const boris_drone::Pose3D& pose2)
{
  //OpenCV method
  double angleThresh = PI/40;

  /* get coordinates */
  cv::Mat cam2world1, cam2world2, origin1, origin2;
  getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  getCameraPositionMatrices(pose2, cam2world2, origin2, true);


  cv:: Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin1.at<double>(0,0),
                                           0, 1, 0, -origin1.at<double>(1,0),
                                           0, 0, 1, -origin1.at<double>(2,0));
  cv:: Mat T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin2.at<double>(0,0),
                                           0, 1, 0, -origin2.at<double>(1,0),
                                           0, 0, 1, -origin2.at<double>(2,0));

  cv:: Mat K2 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                           0,     529.1, 182.2,
                                           0,     0,     1     );
  cv:: Mat K1 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                           0,     529.1, 182.2,
                                           0,     0,     1     );


  std::vector<cv::Point2d> cam0pnts;
  std::vector<cv::Point2d> cam1pnts;

  cv::Mat cam0 = K1*cam2world1.t()*T1;
  cv::Mat cam1 = K2*cam2world2.t()*T2;

  cam0pnts.push_back(pt1);
  cam1pnts.push_back(pt2);

  cv::Mat pnts3D(4,cam0pnts.size(),CV_64F);

  cv::triangulatePoints(cam0,cam1,cam0pnts,cam1pnts,pnts3D);

  pt_out.x = pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0);
  pt_out.y = pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0);
  pt_out.z = pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0);

  cv::Mat reproj1 = cam0*pnts3D;
  cv::Mat reproj2 = cam1*pnts3D;
  double pt1_rx, pt1_ry,pt2_rx, pt2_ry,d1,d2;
  pt1_rx = reproj1.at<double>(0,0)/reproj1.at<double>(2,0);
  pt1_ry = reproj1.at<double>(1,0)/reproj1.at<double>(2,0);
  pt2_rx = reproj2.at<double>(0,0)/reproj2.at<double>(2,0);
  pt2_ry = reproj2.at<double>(1,0)/reproj2.at<double>(2,0);
  d1 = (pt1.x - pt1_rx)*(pt1.x - pt1_rx) + (pt1.y - pt1_ry)*(pt1.y - pt1_ry);
  d2 = (pt2.x - pt2_rx)*(pt2.x - pt2_rx) + (pt2.y - pt2_ry)*(pt2.y - pt2_ry);
  ROS_INFO("Triangulation:");
  ROS_INFO("\t pt_in1 = %f, %f",pt1.x,pt1.y);
  ROS_INFO("\t pt_in2 = %f, %f",pt2.x,pt2.y);
  ROS_INFO("\t pt_rp1 = %f, %f",pt1_rx,pt1_ry);
  ROS_INFO("\t pt_rp2 = %f, %f",pt2_rx,pt2_ry);
  ROS_INFO("\t Dist = %f",d1+d2);
  ROS_INFO("\t Point_out: x=%f, y=%f, z=%f",pt_out.x,pt_out.y,pt_out.z);

//  if (d1+d2 > 550.0)
//  {
//    ROS_INFO("rejected: %f",d1+d2);
//    return false;
//  }
  return true;
}


struct ConstCameraReprojectionError {
  ConstCameraReprojectionError(double observed_x, double observed_y, double* camera)
      : observed_x(observed_x), observed_y(observed_y), camera(camera) {}
  template <typename T>
  bool operator()(const T* const point,
                  T* residuals) const {
    T cam2world[9];              //rotation matrix
    T camera_axis_angles[3];     //angle-axis rotation
    T p_cam[3];
    T p[3];
    //Convert
    ceres::EulerAnglesToRotationMatrix(camera, 3, cam2world);
    ceres::RotationMatrixToAngleAxis(cam2world, camera_axis_angles);



    // Templated pinhole camera model for used with Ceres.  The camera is
    // constant,
    struct ConstCameraReprojectionError {
      ConstCameraReprojectionError(double observed_x, double observed_y, double camera_r0, double camera_r1, double camera_r2,
                                                                         double camera_t0, double camera_t1, double camera_t2)
          : observed_x(observed_x), observed_y(observed_y)
          , camera_r0(camera_r0), camera_r1(camera_r1), camera_r2(camera_r2)
          , camera_t0(camera_t0), camera_t1(camera_t1), camera_t2(camera_t2) {}
      template <typename T>
      bool operator()(const T* const point,
                      T* residuals) const {
        double camera_rot[3] = {camera_r0, camera_r1, camera_r2};
        double camera_tra[3] = {camera_t0, camera_t1, camera_t2};
        double cam2world[9];              //rotation matrix
        double camera_axis_angles[3];     //angle-axis rotation
        T      cam_axis_angles_T[3];
        T p_cam[3];
        T p[3];
        //Convert
        ceres::EulerAnglesToRotationMatrix(camera_rot, 3, cam2world);
        ceres::RotationMatrixToAngleAxis(cam2world, camera_axis_angles);

        //Translate point to get location from camera origin
        p_cam[0] = point[0] - camera_tra[0];
        p_cam[1] = point[1] - camera_tra[1];
        p_cam[2] = point[2] - camera_tra[2];

        cam_axis_angles_T = camera_axis_angles;
        //Get rotated point. -1 because this is the rotation of cam2world (we need world2cam)
        ceres::AngleAxisRotatePoint(cam_axis_angles_T, p_cam, p);

        T predicted_x = 350.6 + (529.1*p[0] / p[2]);
        T predicted_y = 182.2 + (529.1*p[1] / p[2]);

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        return true;
      }
      // Factory to hide the construction of the CostFunction object from
      // the client code.

      static ceres::CostFunction* Create(const double observed_x,
                                         const double observed_y,
                                         const double camera_r0,
                                         const double camera_r1,
                                         const double camera_r2,
                                         const double camera_t0,
                                         const double camera_t1,
                                         const double camera_t2) {
        return (new ceres::AutoDiffCostFunction<ConstCameraReprojectionError, 2, 3>(
                    new ConstCameraReprojectionError(observed_x, observed_y, camera_r0, camera_r1, camera_r2, camera_t0, camera_t1, camera_t2)));
      }
      double observed_x;
      double observed_y;
      double camera_r0;
      double camera_r1;
      double camera_r2;
      double camera_t0;
      double camera_t1;
      double camera_t2;
    };
