#include <boris_drone/map/map.h>

Map::Map()
{
}

Map::Map(ros::NodeHandle* nh, bool do_search, bool stop_if_lost, cv::Mat camera_matrix_K)
                          : cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF

  nh = nh;
  bundle_channel = nh->resolveName("bundle");
  bundle_pub     = nh->advertise<boris_drone::BundleMsg>(bundle_channel, 1);
    // get launch parameters
  this->do_search = do_search;
  this->stop_if_lost = stop_if_lost;
  this->camera_matrix_K = camera_matrix_K;

  // initialize default status boolean
  this->tracking_lost = false;

  // define some threshold used later
  // better if defined in the launch file
  threshold_lost = 10;
  threshold_new_keyframe = 50;
  threshold_new_keyframe_percentage = 0.25;

  // get camera parameters in launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  // initialize empty opencv vectors
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);

  this->initPlanes();

  ROS_DEBUG("map initialized");
}

Map::~Map()
{
}

void Map::resetList()
{
  for (int i = 0; i < this->keyframes.size(); i++)
  {
    delete this->keyframes[i];  // this line calls keyframe destructor
  }
  this->keyframes.clear();
}

void Map::resetPose()
{
  this->resetList();  // remove all keyframes
  // empty the cloud
  this->cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBSIFT> >(new pcl::PointCloud<pcl::PointXYZRGBSIFT>);
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
}

void Map::newKeyframe(const Frame& frame)
{
  reference_keyframe = new Keyframe(cloud, &descriptors, frame);
  this->keyframes.push_back(reference_keyframe);
  int nkeyframes = keyframes.size();
  ROS_INFO("Created new Keyframe. There are %d keyframes",nkeyframes);
  if (nkeyframes > 1)
  {
    std::vector<cv::Point3d> points3D;
    std::vector<int> matching_indices_1;
    std::vector<int> matching_indices_2;
    reference_keyframe->match(*keyframes[nkeyframes-2],points3D,matching_indices_1, matching_indices_2);
    //bundle adjustment
    std::vector<Keyframe*> kfs;
    kfs.push_back(reference_keyframe);
    kfs.push_back(keyframes[nkeyframes-2]);
    doBundleAdjustment(keyframes, false); //false also wroks well!!
  }
}


bool Map::processFrame(const Frame& frame, boris_drone::Pose3D& PnP_pose)
{
  int PnP_result = doPnP(frame, PnP_pose);
  if (keyframeNeeded(frame.pose))
    newKeyframe(frame);
  switch(PnP_result){
    case 1  : //PnP successful
      if (abs(PnP_pose.z - frame.pose.z) > 0.8)
      {
        ROS_INFO("TRACKING LOST ! (PnP found bad symmetric clone?)");
        return false;
      }
      ROS_INFO_THROTTLE(1,"publishing pnp_pose. pnp_pose is: x = %f, y = %f, z = %f",PnP_pose.x,PnP_pose.y,PnP_pose.z);
      return true;
    case -1  : //Empty frame
      ROS_INFO_THROTTLE(3,"Frame is empty");
      return false;
    case -2  : //Empty map
      ROS_INFO_THROTTLE(3,"Map is empty");
      return false;
    case -3  :
      ROS_INFO("Tracking lost! not enough matching points");
      return false;
    case -4  :
      ROS_INFO("Tracking lost! not enough inliers");
      return false;
    case -5 :
      ROS_DEBUG("TRACKING LOST ! (Determinant of rotation matrix)");
      return false;
    default :
      ROS_INFO("ERROR: Invalid return code from doPnP");
      return false;
  }
}


void Map::newReferenceKeyframe(const Frame& current_frame, boris_drone::Pose3D PnP_pose, bool PnP_success)
{
  //b4: check amonst old KF before making new one
  //now immediately make new KF as all KP were used

  boris_drone::Pose3D pose;     // pose used locally to attach to new keyframe
  if (!PnP_success)
  {
    pose = current_frame.pose;  // sensor based pose of current frame
  }
  else
  {
    pose = PnP_pose;               // PnP pose of the current frame
    pose.z = current_frame.pose.z; // in theory, absolute (because ultrasonic sensor)
    // and more precise than PnP, so use this one to map. In practice: Kalman filtered on board
    // (firmware), so, good meausure (not drifted) but not absolute!!! causes bad errors in map.
    // We want to avoid this, so if PnP available, use it!
    pose.rotX = current_frame.pose.rotX;  // idem
    pose.rotY = current_frame.pose.rotY;  // idem
  }

  ROS_INFO("About to make a new Keyframe");
  this->reference_keyframe = new Keyframe(cloud,&descriptors,pose,current_frame);
  this->keyframes.push_back(reference_keyframe);
  ROS_INFO("Made new keyframe. There are %lu keyframes",this->keyframes.size());

}


void Map::doBundleAdjustment(std::vector<Keyframe*> kfs, bool fixed_poses)
{
  ROS_INFO("entered Bundle Adjustment");
  int ncam, npt, nobs, npt_cam, i, j, k, ptIdx, pos, kf, idx_in_kf;
  std::set<int> ba_points;
  std::set<int>::iterator ba_pts_it;
  std::vector< std::vector<int> > kfs_point; //kfs (inner vec) seeing point (outer vec)
  std::vector< std::vector<int> > idx_point_in_kf; //index of the point in corresponding keyframe
  std::pair<std::set<int>::iterator,bool> ouputOfInsert;

  ncam = kfs.size();
  for(i = 0; i<ncam; i++)
  {
    npt_cam = kfs[i]->npts;
    for(j = 0; j < npt_cam; j++)
    {
      if (kfs[i]->pointIsMapped[j]) //Only take points that are mapped
      {
        ptIdx = kfs[i]->points[j]; //This is the index of the point in the map
        ouputOfInsert = ba_points.insert(ptIdx);
        //pos is the position where the point was inserted,
        //or the position where it already was
        pos = std::distance(ba_points.begin(), ouputOfInsert.first);
        if (ouputOfInsert.second==false) // ptIdx was already in ba_points
        {
          kfs_point[pos].push_back(i); //Add this kf to the kfs seeing this pt
          idx_point_in_kf[pos].push_back(j); //idx of the pt in the kf
        }
        else //ptIdx wasn't in ba_points before the insert
        {
          std::vector<int> kf_thispoint(1,i); //vector containing 1 element: i
          std::vector<int> idx_thispt_in_thiskf(1,j); //index of this point in this kf
          kfs_point.insert(kfs_point.begin() + pos, kf_thispoint);
          idx_point_in_kf.insert(idx_point_in_kf.begin() + pos, idx_thispt_in_thiskf);
        }
      }
    }
  }
  ROS_INFO("Preparing bundle. N_ba_points = %lu", ba_points.size());
  ROS_INFO("check: %lu = %lu", ba_points.size(), kfs_point.size());
  ba_pts_it = ba_points.begin();
  for(i = 0; i<ba_points.size(); i++)
  {
    ROS_INFO("Point %d seen by:",*ba_pts_it);
    for (j = 0; j<kfs_point[i].size();j++)
    {
      ROS_INFO("%d",kfs_point[i][j]);
    }
    ba_pts_it++;
  }
  ROS_INFO("Done making lists of point observations. Now removing single obserations");
  ROS_INFO("Check sizes : %lu = %lu", ba_points.size(),kfs_point.size());
  //Remove points that are only seen once and count observations
  i = kfs_point.size();
  nobs = 0;
  for (ba_pts_it = ba_points.end(); ba_pts_it != ba_points.begin(); )
  {
    i--;
    if(kfs_point[i].size() < 2)
    {
      ROS_INFO("About to erase point %d",i);
      ba_points.erase(ba_pts_it--);
      kfs_point.erase(kfs_point.begin() + i);
      idx_point_in_kf.erase(idx_point_in_kf.begin() + i);
      ROS_INFO("Erased point %d",i);
    }
    else
    {
      ba_pts_it--;
      ROS_INFO("Not erasing point %d. It was seen by %lu kfs",i,kfs_point[i].size());
      nobs += kfs_point[i].size();
      ROS_INFO("Total nobs = %d",nobs);
    }
  }
  ROS_INFO("Done removing. There are %d observations", nobs);

  /*
  print set info
  ROS_INFO("Preparing bundle. N_ba_points = %lu", ba_points.size());
  ROS_INFO("check: %lu = %lu", ba_points.size(), kfs_point.size());
  ba_pts_it = ba_points.begin();
  for(i = 0; i<ba_points.size(); i++)
  {
    ROS_INFO("Point %d seen by:",*ba_pts_it);
    for (j = 0; j<kfs_point[i].size();j++)
    {
      ROS_INFO("%d",kfs_point[i][j]);
    }
    ba_pts_it++;
  }

  */

  boris_drone::BundleMsg::Ptr msg(new boris_drone::BundleMsg);
  npt = ba_points.size();
  msg->fixed_poses = fixed_poses;
  msg->num_cameras = ncam;
  msg->num_points  = npt;
  msg->num_observations = nobs;
  msg->observations.resize(nobs);
  msg->points.resize(npt);
  ba_pts_it = ba_points.begin();
  k = 0;
  for (i = 0; i < npt; ++i) {
    for (j = 0; j < kfs_point[i].size(); ++j) {
      //ROS_INFO("recording observation %d in message",k);
      kf = kfs_point[i][j];
      idx_in_kf = idx_point_in_kf[i][j];
      msg->observations[k].camera_index = kf;
      msg->observations[k].point_index  = i;
      msg->observations[k].x            = kfs[kf]->imgPoints[idx_in_kf].x;
      msg->observations[k].y            = kfs[kf]->imgPoints[idx_in_kf].y;
      k++;
    }
    //ROS_INFO("recording point %d in message",i);

    msg->points[i].x = cloud->points[*ba_pts_it].x;
    msg->points[i].y = cloud->points[*ba_pts_it].y;
    msg->points[i].z = cloud->points[*ba_pts_it].z;
    ba_pts_it++;
  }
  msg->cameras.resize(ncam);
  for (i = 0; i < ncam; ++i) {
    //ROS_INFO("recording cam %d in message",i);
    tf::Matrix3x3 drone2world, cam2drone, cam2world;
    double roll, pitch, yaw;
    drone2world.setRPY(kfs[i]->pose.rotX,kfs[i]->pose.rotY,kfs[i]->pose.rotZ);
    cam2drone.setRPY(-PI/2, 0, -PI/2);
    cam2world = drone2world*cam2drone;
    cam2world.getRPY(roll, pitch, yaw);
    /*
    Camera info
    cout << "=== Camera number " << i << " ==="<< std::endl;
    cout << "drone2world RPY" << std::endl;
    cout <<  kfs[i]->pose.rotX << ";  " << kfs[i]->pose.rotY << ";  " << kfs[i]->pose.rotZ  << std::endl;
    cout << "cam2drone RPY" << std::endl;
    cout <<  -PI/2 << ";  " << 0 << ";  " << -PI/2  << std::endl;
    cout << "RPY" << std::endl;
    cout <<  roll << ";  " << pitch << ";  " << yaw  << std::endl;
    */
    msg->cameras[i].rotX = roll;
    msg->cameras[i].rotY = pitch;
    msg->cameras[i].rotZ = yaw;
    msg->cameras[i].x = kfs[i]->pose.x;
    msg->cameras[i].y = kfs[i]->pose.y;
    msg->cameras[i].z = kfs[i]->pose.z;

  }
  bundle_pub.publish(*msg);
}


int Map::doPnP(const Frame& current_frame, boris_drone::Pose3D& PnP_pose)
{
  std::vector<cv::Point3f> inliers_map_matching_points;
  std::vector<cv::Point2f> inliers_frame_matching_points;
  int result = matchWithFrame(current_frame, inliers_map_matching_points, inliers_frame_matching_points);
  if (result < 0)
    return result;

  cv::Mat_<double> tcam, cam2world, world2drone, distCoeffs;
  distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

  // solve with PnP n>3
  cv::solvePnP(inliers_map_matching_points, inliers_frame_matching_points,
               this->camera_matrix_K, distCoeffs, rvec, tvec, true, CV_EPNP);

  cv::Rodrigues(rvec, cam2world);
  if (fabs(determinant(cam2world)) - 1 > 1e-07)
    return -5;

  //front camera:
  cv::Mat drone2cam = rollPitchYawToRotationMatrix(-PI/2, 0, -PI / 2);
  //bottom camera:
  //cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI,   0, -PI / 2);

  tcam = -cam2world.t() * tvec;
  PnP_pose.x = tcam(0);
  PnP_pose.y = tcam(1);
  PnP_pose.z = tcam(2);

  cv::Mat_< double > world2cam = cam2world.t();
  cv::Mat_< double > cam2drone = drone2cam.t();
  world2drone = world2cam * cam2drone;

  tf::Matrix3x3(world2drone(0, 0), world2drone(0, 1), world2drone(0, 2),
  world2drone(1, 0), world2drone(1, 1), world2drone(1, 2),
  world2drone(2, 0), world2drone(2, 1), world2drone(2, 2))
  .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

  PnP_pose.xvel = 0.0;
  PnP_pose.yvel = 0.0;
  PnP_pose.zvel = 0.0;
  PnP_pose.rotXvel = 0.0;
  PnP_pose.rotYvel = 0.0;
  PnP_pose.rotZvel = 0.0;

  PnP_pose.header.stamp = current_frame.pose.header.stamp;  // needed for rqt_plot

  ROS_INFO_THROTTLE(3,"Did PnP. There are %lu inliers", inliers_map_matching_points.size());
  return 1;
}

void Map::initPlanes()
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

bool Map::keyframeNeeded(boris_drone::Pose3D pose)
{
  double thresh = 0.25;
  if (keyframes.size() == 0)
    return true;
  else
    return (getSqDist(pose, reference_keyframe->pose) > thresh);
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%lu] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}


int Map::matchWithFrame(const Frame& frame, std::vector<cv::Point3f>& inliers_map_matching_points,
                                             std::vector<cv::Point2f>& inliers_frame_matching_points)
{
  if (frame.descriptors.rows == 0)
    return -1;
  if (this->descriptors.rows == 0)
    return -2;

  std::vector<cv::Point3f> map_matching_points;
  std::vector<cv::Point2f> frame_matching_points;
  std::vector<int> map_indices, frame_indices, inliers;
  pcl::PointXYZRGBSIFT pcl_point;
  matchDescriptors(this->descriptors, frame.descriptors, map_indices, frame_indices);
  if (map_indices.size() < threshold_lost)
    return -3;

  for (unsigned k = 0; k < map_indices.size(); k++)
  {
      pcl_point = this->cloud->points[map_indices[k]];
      cv::Point3f map_point(pcl_point.x,pcl_point.y,pcl_point.z);
      map_matching_points.push_back(map_point);
      frame_matching_points.push_back(frame.imgPoints[frame_indices[k]]);
  }

  cv::Mat distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);
  cv::solvePnPRansac(map_matching_points, frame_matching_points, this->camera_matrix_K, distCoeffs, rvec, tvec,
                     true, 2500, 2, 2 * threshold_new_keyframe, inliers, CV_P3P);  // or: CV_EPNP and CV_ITERATIVE
  if (inliers.size() < threshold_lost)
    return -4;

  for (int j = 0; j < inliers.size(); j++)
  {
    int i = inliers[j];
    inliers_map_matching_points.push_back(map_matching_points[i]);
    inliers_frame_matching_points.push_back(frame_matching_points[i]);
  }
  return 1;
}


void Map::updateBundle(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  int num_points = bundlePtr->num_points;
  ROS_INFO("Update Bundle");

  for (int i = 0; i < num_points; ++i) {
    pcl::PointXYZRGBSIFT new_point;
    new_point.x = bundlePtr->points[i].x;
    new_point.y = bundlePtr->points[i].y;
    new_point.z = bundlePtr->points[i].z;
    cloud->points[i] = new_point;
  }
}
