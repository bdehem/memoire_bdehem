/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/mapping_node.h>

MappingNode::MappingNode()
  : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
  , cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF

  // initialize default status boolean
  this->tracking_lost = false;
  this->pending_reset = false;

  // get launch parameters
  ros::param::get("~do_search", this->do_search);
  ros::param::get("~stop_if_lost", this->stop_if_lost);

  // define some threshold used later
  // better if defined in the launch file
  threshold_lost = 10;
  threshold_new_keyframe = 50;
  threshold_new_keyframe_percentage = 0.25;

  // initialize an Empty reference keyframe
  reference_keyframe = new Keyframe(this);
  this->keyframes.push_back(reference_keyframe);
  ROS_INFO("Building MappingNode, there are %lu keyframes",this->keyframes.size());

  // Subsribers
  strategy_channel = nh.resolveName("strategy");
  strategy_sub     = nh.subscribe(strategy_channel, 10, &MappingNode::strategyCb, this);
  processed_image_channel = nh.resolveName("processed_image");
  processed_image_sub     = nh.subscribe(processed_image_channel, 1, &MappingNode::processedImageCb,this);
  reset_pose_channel = nh.resolveName("reset_pose");
  reset_pose_sub     = nh.subscribe(reset_pose_channel, 1, &MappingNode::resetPoseCb, this);
  end_reset_pose_channel = nh.resolveName("end_reset_pose");
  end_reset_pose_sub     = nh.subscribe(end_reset_pose_channel, 1, &MappingNode::endResetPoseCb, this);

  // Publishers
  pose_visual_channel = nh.resolveName("pose_visual");
  pose_visual_pub     = nh.advertise< boris_drone::Pose3D >(pose_visual_channel, 1);
  pose_correction_channel = nh.resolveName("pose_visual_correction");
  pose_correction_pub     = nh.advertise< boris_drone::Pose3D >(pose_correction_channel, 1);
  target_channel = nh.resolveName("boris_drone/target_detected");
  target_pub     = nh.advertise< boris_drone::TargetDetected >(target_channel, 1);

  // initialize the map and the visualizer
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZRGBSIFT > single_color(cloud, 0, 255, 0);
  visualizer->setBackgroundColor(0, 0.1, 0.3);
  visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, single_color, "SIFT_cloud");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "SIFT_cloud");
  visualizer->addCoordinateSystem(1.0);  // red: x, green: y, blue: z

  // get camera parameters in launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  this->camera_matrix_K =
        (cv::Mat_< double >(3, 3) << Read::focal_length_x(), 0,                      Read::img_center_x(),
                                     0,                      Read::focal_length_y(), Read::img_center_y(),
                                     0,                      0,                      1                   );

  // initialize empty opencv vectors
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);

  this->init_planes();

  ROS_DEBUG("simple_map initialized");
}

MappingNode::~MappingNode()
{
}


void MappingNode::resetList()
{
  for (int i = 0; i < this->keyframes.size(); i++)
  {
    delete this->keyframes[i];  // this line calls keyframe destructor
  }
  this->keyframes.clear();
}


void MappingNode::strategyCb(const std_msgs::Int16::ConstPtr strategyPtr)
{
  strategy = strategyPtr->data;
}

void MappingNode::resetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = true;
  this->resetList();  // remove all keyframes
  this->cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBSIFT> >(
      new pcl::PointCloud<pcl::PointXYZRGBSIFT>);  // empty the cloud
  processed_image_sub.shutdown();                    // stop receiving new visual information
  // update visualizer
  this->visualizer->updatePointCloud<pcl::PointXYZRGBSIFT>(this->cloud, "SIFT_cloud");
  // reset
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  processed_image_sub = nh.subscribe(processed_image_channel, 3, &MappingNode::processedImageCb, this);
}

void MappingNode::endResetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = false;
}

void MappingNode::processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  if (pending_reset || strategy == WAIT || strategy == TAKEOFF)
    return;
  this->lastProcessedImgReceived = processed_image_in;
  Frame current_frame(processed_image_in);

  if (this->cloud->size() != 0)  // IF THE MAP IS NOT EMPTY
  {
    std::vector<int> inliers;
    boris_drone::Pose3D PnP_pose;

    bool PnP_success = this->doPnP(current_frame, PnP_pose, inliers, reference_keyframe);
    // filter out bad clone configuration resulting in negative altitude
    if (abs(PnP_pose.z - current_frame.pose.z) > 0.8)  // treshold in cm
    {
      ROS_INFO("TRACKING LOST ! (PnP found bad symmetric clone?)");
      PnP_success = false;
    }

    if (PnP_success)
    {
      this->publishPoseVisual(current_frame.pose, PnP_pose);
    }

    // if a new keyframe is needed (because the current one does not match enough, search among
    // previous ones, or create a new one)
    if (this->newKeyframeNeeded(inliers.size()))
    {
      this->newReferenceKeyframe(current_frame,PnP_pose,PnP_success);
      return;
    }
  }
  else if (current_frame.imgPoints.size()!=0) // IF THE MAP IS EMPTY BUT NOT THE FRAME
  {
    ROS_DEBUG("MAP INITIALIZATION");
    reference_keyframe = new Keyframe(this,current_frame.pose,current_frame);
    this->keyframes.push_back(reference_keyframe);
    ROS_INFO("initilizing map. There are %lu keyframes",this->keyframes.size());
    this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  }
  ROS_DEBUG("Present keyframe id = %d", reference_keyframe->getID());
}

void MappingNode::newReferenceKeyframe(Frame current_frame, boris_drone::Pose3D PnP_pose, bool PnP_success)
{
  //b4: check amonst old KF before making new one
  //now immediately make new KF as all KP were used
  int keyframe_ID = -1;

  boris_drone::Pose3D pose;     // pose used locally to attach to new keyframe
  if (!PnP_success)
  {
    pose = current_frame.pose;  // sensor based pose of current frame
  }
  else
  {
    pose = PnP_pose;                    // PnP pose of the current frame
    pose.z = current_frame.pose.z;  // in theory, absolute (because ultrasonic sensor)
    // and more precise than PnP, so use this one to map. In practice: Kalman filtered on board
    // (firmware), so, good meausure (not drifted) but not absolute!!! causes bad errors in map.
    // We want to avoid this, so if PnP available, use it!
    pose.rotX = current_frame.pose.rotX;  // idem
    pose.rotY = current_frame.pose.rotY;  // idem
  }

  //from b4
//  bool search_success = false;
//  if (do_search)
//  {
//    search_success = closestKeyframe(pose, keyframe_ID, current_frame);
//  }
//  if (search_success)
//  {
//    this->reference_keyframe = this->keyframes[keyframe_ID];
//  }
//  else
//  {
    ROS_INFO("About to make a new Keyframe");
    this->reference_keyframe = new Keyframe(this,pose,current_frame);
    this->keyframes.push_back(reference_keyframe);
    ROS_INFO("Made new keyframe. There are %lu keyframes",this->keyframes.size());
    this->visualizer->updatePointCloud<pcl::PointXYZRGBSIFT>(this->cloud, "SIFT_cloud");
//  }
}

void MappingNode::doBundleAdjustment()
{
  /****** INPUT DATA *****************/
  std::vector<cv::Point3d> points3D;
  std::vector<std::vector<cv::Point2d> > pointsImg;
  std::vector<std::vector<int> > visibility;
  std::vector<cv::Mat> cameraMatrix, distCoeffs, R, T;
  int NPOINTS = this->cloud->size(); // number of 3d points
  int NCAMS   = this->keyframes.size(); // number of cameras

  // fill 3D points with initial guess (current position of the point)
  points3D.resize(NPOINTS);
  for (int i; i<NPOINTS; i++)
  {
    points3D[i].x = this->cloud->points[i].x;
    points3D[i].y = this->cloud->points[i].y;
    points3D[i].z = this->cloud->points[i].z;
  }

  // fill image projections
  pointsImg.resize(NCAMS);
  visibility.resize(NCAMS);
  distCoeffs.resize(NCAMS);
  cameraMatrix.resize(NCAMS);
  R.resize(NCAMS);
  T.resize(NCAMS);
  for(int i=0; i<NCAMS; i++)
  {
    std::vector<int> this_visibility(NPOINTS, 0);
    pointsImg[i].resize(NPOINTS);
    for (int j=0; j < this->keyframes[i]->points.size();j++)
    {
      int ptIdx = this->keyframes[i]->points[j];
      pointsImg[i][ptIdx]  = this->keyframes[i]->mapped_imgPoints[j];
      this_visibility[ptIdx] = 1;
    }
    visibility[i] = this_visibility;
    distCoeffs[i]   = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    cameraMatrix[i] = (cv::Mat_<double>(3, 3) << Read::focal_length_x(), 0                     , Read::img_center_x(),
                                                 0,                      Read::focal_length_y(), Read::img_center_y(),
                                                 0,                      0,                      1                   );
    getCameraPositionMatrices(this->keyframes[i]->pose,R[i],T[i]);
  }
  /***********************************/

  /****** RUN BUNDLE ADJUSTMENT ******/
  cvsba::Sba sba;

  // change params if desired
  cvsba::Sba::Params params ;
  params.type = cvsba::Sba::MOTIONSTRUCTURE;
  params.fixedIntrinsics = 5;
  params.fixedDistortion = 5;
  sba.setParams(params);
  sba.run(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
  /***********************************/

  // Update pointcloud
  for (int i; i<NPOINTS; i++)
  {
    this->cloud->points[i].x = points3D[i].x;
    this->cloud->points[i].y = points3D[i].y;
    this->cloud->points[i].z = points3D[i].z;
  }
}

void MappingNode::publishPoseVisual(boris_drone::Pose3D poseFrame, boris_drone::Pose3D PnP_pose)
{
  boris_drone::Pose3D pose_correction;
  pose_correction.header.stamp = ros::Time::now();
  pose_correction.x = poseFrame.x - PnP_pose.x;
  pose_correction.y = poseFrame.y - PnP_pose.y;
  pose_correction.z = poseFrame.z - PnP_pose.z;
  pose_correction.rotX = poseFrame.rotX - PnP_pose.rotX;
  pose_correction.rotY = poseFrame.rotY - PnP_pose.rotY;
  pose_correction.rotZ = poseFrame.rotZ - PnP_pose.rotZ;
  pose_visual_pub.publish(PnP_pose);
  pose_correction_pub.publish(pose_correction);
}

bool MappingNode::doPnP(Frame current_frame, boris_drone::Pose3D& PnP_pose, std::vector< int >& inliers,
                Keyframe* ref_keyframe)
{
  std::vector<std::vector<int> > idx_matching_points;
  std::vector<cv::Point3f> map_matching_points;
  std::vector<cv::Point2f> frame_matching_points;

  //b4: ref_keyframe->matchwithframe
  this->matchWithFrame(current_frame, idx_matching_points, map_matching_points, frame_matching_points);
  //from b4
//  if (ref_keyframe_matching_points.size() < threshold_lost)
//  {
//    if (ref_keyframe == reference_keyframe)
//      ROS_INFO("TRACKING LOST ! (Not enough matching points: %lu)",
//               ref_keyframe_matching_points.size());
//    return false;
//  }

  cv::Mat_<double> tcam, cam2world, world2drone, distCoeffs;

  distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

  // solve with P3P
  try{
    cv::solvePnPRansac(map_matching_points, frame_matching_points, this->camera_matrix_K,
                       distCoeffs, rvec, tvec, true, 2500, 2, 2 * threshold_new_keyframe, inliers,
                       CV_P3P);  // alternatives: CV_EPNP and CV_ITERATIVE
  }
  catch(cv::Exception e)
  {
    ROS_INFO("caught cv exception when running ransac");
  }

  if (inliers.size() < threshold_lost)
  {
    if (ref_keyframe == reference_keyframe)
      ROS_INFO("TRACKING LOST ! (Not enough inliers)");
    return false;
  }

  //fromb4
//  if (ref_keyframe == reference_keyframe)
//  {
    // select only inliers
    std::vector<cv::Point3f> inliers_map_matching_points;
    std::vector<cv::Point2f> inliers_frame_matching_points;

    for (int j = 0; j < inliers.size(); j++)
    {
      int i = inliers[j];

      inliers_map_matching_points.push_back(map_matching_points[i]);
      inliers_frame_matching_points.push_back(frame_matching_points[i]);
    }

    // solve with PnP n>3
    cv::solvePnP(inliers_map_matching_points, inliers_frame_matching_points,
                 this->camera_matrix_K, distCoeffs, rvec, tvec, true, CV_EPNP);
//  }

  cv::Rodrigues(rvec, cam2world);

  if (fabs(determinant(cam2world)) - 1 > 1e-07)
  {
    ROS_DEBUG("TRACKING LOST ! (Determinant of rotation matrix)");
    return false;
  }

  // equivalent to rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
  cv::Mat_< double > drone2cam =
      (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

  tcam = -cam2world.t() * tvec;
  PnP_pose.x = tcam(0);
  PnP_pose.y = tcam(1);
  PnP_pose.z = tcam(2);

  cv::Mat_< double > world2cam = cam2world.t();
  cv::Mat_< double > cam2drone = drone2cam.t();
  world2drone = world2cam * cam2drone;

  tf::Matrix3x3(world2drone(0, 0), world2drone(0, 1), world2drone(0, 2), world2drone(1, 0),
                world2drone(1, 1), world2drone(1, 2), world2drone(2, 0), world2drone(2, 1),
                world2drone(2, 2))
      .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

  PnP_pose.xvel = 0.0;
  PnP_pose.yvel = 0.0;
  PnP_pose.zvel = 0.0;
  PnP_pose.rotXvel = 0.0;
  PnP_pose.rotYvel = 0.0;
  PnP_pose.rotZvel = 0.0;

  PnP_pose.header.stamp = current_frame.pose.header.stamp;  // needed for rqt_plot

  return true;
}

//! better if in the future, PnP is used, for the moment, detection with 2D hypothesis.
void MappingNode::targetDetectedPublisher()
{
  if (lastProcessedImgReceived && target_pub.getNumSubscribers() > 0)
  {
    if (lastProcessedImgReceived->target_detected)
    {
      ROS_DEBUG("TARGET IS DETECTED");

      boris_drone::TargetDetected msg;
      msg.pose = lastProcessedImgReceived->pose;
      msg.img_point.x = lastProcessedImgReceived->target_points[4].x;
      msg.img_point.y = lastProcessedImgReceived->target_points[4].y;
      msg.img_point.z = 0;
      std::vector< cv::Point2f > target_center(1);
      target_center[0].x = lastProcessedImgReceived->target_points[4].x;
      target_center[0].y = lastProcessedImgReceived->target_points[4].y;
      std::vector< cv::Point3f > world_coord;
      projection_2D(target_center, msg.pose, world_coord);
      msg.world_point.x = world_coord[0].x;
      msg.world_point.y = world_coord[0].y;
      msg.world_point.z = world_coord[0].z;
      target_pub.publish(msg);
    }
  }
}

void MappingNode::init_planes()
{
  float thres = 0.95;
  cv::Mat top_left = (cv::Mat_< double >(3, 1) << -Read::img_center_x() / Read::focal_length_x(),
                      -Read::img_center_y() / Read::focal_length_y(), thres);
  cv::Mat top_right = (cv::Mat_< double >(3, 1)
                           << (Read::img_width() - Read::img_center_x()) / Read::focal_length_x(),
                       -Read::img_center_y() / Read::focal_length_y(), thres);
  cv::Mat bottom_right =
      (cv::Mat_< double >(3, 1) << (Read::img_width() - Read::img_center_x()) /
                                       Read::focal_length_x(),
       (Read::img_height() - Read::img_center_y()) / Read::focal_length_y(), thres);
  cv::Mat bottom_left =
      (cv::Mat_< double >(3, 1) << -Read::img_center_x() / Read::focal_length_x(),
       (Read::img_height() - Read::img_center_y()) / Read::focal_length_y(), thres);
  cam_plane_top = top_left.cross(top_right);
  cam_plane_right = top_right.cross(bottom_right);
  cam_plane_bottom = bottom_right.cross(bottom_left);
  cam_plane_left = bottom_left.cross(top_left);
  ROS_DEBUG("MAP: planes initilized");
}

bool customLess(std::vector< int > a, std::vector< int > b)
{
  return a[1] > b[1];
}

bool MappingNode::closestKeyframe(const boris_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame)
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

void MappingNode::getVisibleKeyframes(const boris_drone::Pose3D& pose, std::vector<std::vector<int> >& keyframes_ID)
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

void MappingNode::getVisiblePoints(const boris_drone::Pose3D& pose, std::vector<int>& idx)
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

bool MappingNode::newKeyframeNeeded(int number_of_common_keypoints)
{
  return newKeyframeNeeded(number_of_common_keypoints, this->reference_keyframe);
}

bool MappingNode::newKeyframeNeeded(int number_of_common_keypoints, Keyframe* reference_keyframe_candidate)
{
  return (number_of_common_keypoints < threshold_new_keyframe);
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%lu] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}

//TODO idx_matching_points useless?
void MappingNode::matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                         std::vector<cv::Point3f>& map_matching_points,
                         std::vector<cv::Point2f>& frame_matching_points)
{
  if (frame.descriptors.rows == 0 || this->descriptors.rows == 0)
  {
    return;
  }
  std::vector<cv::DMatch> simple_matches;
  matcher.match(frame.descriptors, this->descriptors, simple_matches);

  // threshold test
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      std::vector<int> v(2);
      v[0] = simple_matches[k].trainIdx;
      v[1] = simple_matches[k].queryIdx;
      idx_matching_points.push_back(v);

      pcl::PointXYZRGBSIFT pcl_point = this->cloud->points[simple_matches[k].trainIdx];
      cv::Point3f map_point(pcl_point.x,pcl_point.y,pcl_point.z);

      map_matching_points.push_back(map_point);
      frame_matching_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
    }
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_map");
  ROS_INFO_STREAM("simple map started!");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  MappingNode map;
  ros::Time t = ros::Time::now() + ros::Duration(13);
  while (ros::Time::now() < t)
  {
    map.visualizer->spinOnce(100);
  }

  ros::Rate r(3);

  int visualizer_count = 0;

  while (ros::ok())
  {
    map.visualizer->spinOnce(10);

    map.targetDetectedPublisher();

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
