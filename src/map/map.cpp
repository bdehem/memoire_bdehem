#include <boris_drone/map/map.h>

int Map::point_ID_counter = 0;

Map::Map() {}

Map::Map(ros::NodeHandle* nh, cv::Mat camera_matrix_K)
                          : cloud(new pcl::PointCloud< pcl::PointXYZ >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF

  nh = nh;
  bundle_channel = nh->resolveName("bundle");
  bundle_pub     = nh->advertise<boris_drone::BundleMsg>(bundle_channel, 1);
    // get launch parameters
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
    ROS_ERROR("cam_matrix not properly transmitted");
  if (!Read::ImgSizeParams("img_size"))
    ROS_ERROR("img_size not properly transmitted");

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
  std::map<int, Keyframe*>::iterator iter;
  for (iter = keyframes.begin(); iter != keyframes.end(); ++iter)
  {
    delete iter->second;  // this line calls keyframe destructor
  }
  keyframes.clear();
}

void Map::resetPose()
{
  this->resetList();  // remove all keyframes
  // empty the cloud
  this->cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
}

int Map::addPoint(cv::Point3d& coordinates, cv::Mat descriptor)
{
  int ID = point_ID_counter++;
  pcl::PointXYZ new_point;
  new_point.x = coordinates.x;
  new_point.y = coordinates.y;
  new_point.z = coordinates.z;
  cloud->points.push_back(new_point);
  descriptors.push_back(descriptor);
  pt_idxes[ID] = pt_IDs.size();
  pt_IDs.push_back(ID);
  return ID;
}

void Map::updatePoint(int pt_ID, pcl::PointXYZ new_point)
{
  if (pt_idxes[pt_ID] >= pt_IDs.size())
  {
    ROS_INFO("Warning: point to be updated does not exist");
    return;
  }
  cloud->points[pt_idxes[pt_ID]] = new_point;
}

void Map::removePoint(int ptID)
{
  ROS_INFO("enter removePoint");
  int index = pt_idxes[ptID];
  if (index >= pt_IDs.size())
    return;
  std::set<int> kfs = keyframes_seeing_point[ptID];
  std::set<int>::iterator it;
  for (it = kfs.begin();it!=kfs.end();++it)
  {
    keyframes[*it]->removePoint(ptID);
    points_seen_by_keyframe[*it].erase(ptID);
  }
  keyframes_seeing_point.erase(ptID);
  pt_idxes.erase(ptID);
  pt_IDs.erase(pt_IDs.begin()+index);
  cloud->erase(cloud->begin()+index);
  ROS_INFO("middle removePoint");
  // Removing a row of descriptors TODO: do something similar as in keyframe?
  cv::Mat temp, roi;    // Matrix of which a row will be deleted.
  if ( index > 0 ) // Copy everything above that one row.
  {
    cv::Rect rect( 0, 0, descriptors.cols, index );
    roi = descriptors( rect );
    temp = roi.clone();
  }
  if ( index < descriptors.rows - 1 ) // Copy everything below that one row.
  {
    cv::Rect rect1( 0, index+1, descriptors.cols, descriptors.rows - index - 1);
    roi = descriptors( rect1 );
    temp.push_back(roi);
  }
  descriptors = temp;
  std::map<int,int>::iterator it2;
  for(it2 = pt_idxes.begin();it2!=pt_idxes.end();++it2)
    if (it2->second > index)
      it2->second--;
  ROS_INFO("finished removePoint");
}

void Map::setPointAsSeen(int pt_ID, int kf_ID)
{
  points_seen_by_keyframe[kf_ID].insert(pt_ID);
  keyframes_seeing_point[pt_ID].insert(kf_ID);
}

void Map::matchKeyframes(Keyframe& kf0, Keyframe& kf1)
{
  int pt_ID, i;
  int next_point_ID;
  std::vector<cv::Point3d> points3D;
  std::vector<int> idx_kf0, idx_kf1, match_ID;
  std::vector<bool> point_is_new;
  if (pt_IDs.empty())
    next_point_ID = 0;
  else
    next_point_ID = *(--pt_IDs.end()) + 1;

  match(kf0, kf1, points3D, idx_kf0, idx_kf1, match_ID, point_is_new, next_point_ID);
  for (i = 0; i<match_ID.size(); i++)
  {
    if (point_is_new[i])
    {
      pt_ID = addPoint(points3D[i], kf0.descriptors.rowRange(idx_kf0[i],idx_kf0[i]+1));
      ROS_INFO("check: %d = %d",pt_ID, match_ID[i]);
    }
    else
      pt_ID = match_ID[i];
    setPointAsSeen(pt_ID, kf0.ID);
    setPointAsSeen(pt_ID, kf1.ID);
  }
}

void Map::newKeyframe(const Frame& frame)
{
  Keyframe* old = reference_keyframe;
  reference_keyframe = new Keyframe(cloud, frame);
  if (keyframes.empty())
  {
    keyframes[reference_keyframe->ID] = reference_keyframe;
    return;
  }
  std::vector<int> keyframes_to_adjust;
  std::map<int,Keyframe*>::iterator it;
  for (it = keyframes.begin(); it!=keyframes.end();it++)
  {
    //matchKeyframes(*reference_keyframe, *(it->second)); //Sould be better but results are not great
    ROS_INFO("keyframes_to_adjust: %d",it->first);
    keyframes_to_adjust.push_back(it->first); //add all kfs
  }
  matchKeyframes(*reference_keyframe, *old);
  keyframes_to_adjust.push_back(reference_keyframe->ID);
  ROS_INFO("keyframes_to_adjust: %d",reference_keyframe->ID);
  keyframes[reference_keyframe->ID] = reference_keyframe;

  ROS_INFO("\t Map now has %lu points",this->cloud->points.size());
  ROS_INFO("\t check: %d = %lu", this->descriptors.rows, this->cloud->points.size());
  //removePoint(3);
  doBundleAdjustment(keyframes_to_adjust);
}


bool Map::processFrame(const Frame& frame, boris_drone::Pose3D& PnP_pose, bool keyframeneeded)
{
  int PnP_result = doPnP(frame, PnP_pose);
  if ((keyframeneeded)||(keyframes.size() == 0))
  {
    ROS_INFO("About to make new keyframe");
    newKeyframe(frame);
  }
  switch(PnP_result){
    case 1  : //PnP successful
      if (abs(PnP_pose.z - frame.pose.z) > 0.8)
      {
        ROS_INFO("TRACKING LOST ! (PnP found bad symmetric clone?)");
        return false;
      }
      ROS_INFO_THROTTLE(1,"PnP_pose is: x = %f, y = %f, z = %f",PnP_pose.x,PnP_pose.y,PnP_pose.z);
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

int Map::getPointsSeenByKeyframes(std::vector<int> kf_IDs,
                                 std::map<int,std::map<int,int> >& points)
{
  //Output: points[ID] is a map that maps keyframe IDs of keyframes seeing it to
  //the index of the point in the keyframe

  //First take all points seen by any of the keyframes in kf_IDs
  ROS_INFO("get_pts_start");
  int i, nobs;
  std::set<int> local_point_set;
  std::set<int> kf_set;
  std::set<int> point_set;
  std::set<int>::iterator it1, it2;
  int ncam = kf_IDs.size();
  for (i = 0; i < ncam; ++i)
  {
    local_point_set = points_seen_by_keyframe[kf_IDs[i]];
    for (it1 = local_point_set.begin(); it1 != local_point_set.end(); ++it1)
      point_set.insert(*it1);
  }
  ROS_INFO("Added points");

  //Remove points seen by only one of the keyframes in kf_IDs, and counts obs
  nobs = 0;
  for (it1 = point_set.begin(); it1 != point_set.end(); )
  {
    kf_set = keyframes_seeing_point[*it1];
    std::set<int> intersection;
    std::set_intersection(kf_set.begin(), kf_set.end(),
                          kf_IDs.begin(), kf_IDs.end(),
                          std::inserter(intersection, intersection.end()) );// intersection.begin());
    if(intersection.size() < 2)
    {
      ROS_INFO("erasing point %d", *it1);
      point_set.erase(it1++);
    }
    else
    {
      std::map<int,int> this_point;
      for (it2 = intersection.begin();it2!=intersection.end();++it2)
      {
        int local_index = keyframes[*it2]->point_idx[*it1];
        ROS_INFO("in pointseenbykeyframes. pointID = %d, keyframeID = %d, local idx = %d",*it1,*it2,local_index);
        this_point[*it2] = keyframes[*it2]->point_idx[*it1];
      }
      points[*it1] = this_point;
      it1++;
      nobs += intersection.size();
    }
  }
  ROS_INFO("get_pts_end");
  return nobs;
}

void Map::doBundleAdjustment(std::vector<int> kf_IDs)
{
  ROS_INFO("entered Bundle Adjustment");
  int ncam, npt, nobs, i, j, k;
  std::map<int,bool> is_fixed; //true if we fix position of corresponding kf
  std::map<int,std::map<int,int> > points;
  std::map<int,std::map<int,int> >::iterator points_it;
  std::map<int,int>::iterator inner_it;
  nobs = getPointsSeenByKeyframes(kf_IDs, points);
  ncam = kf_IDs.size();
  npt  = points.size();

  for(i = 0; i < ncam; ++i)
  {
    is_fixed[kf_IDs[i]] = (kf_IDs[i] < ncam/2);
    ROS_INFO("Keyframe %d is fixed: %s !",kf_IDs[i], is_fixed[kf_IDs[i]] ?  "true" : "false");
  }

  /*  print a bunch of things */
  ROS_INFO("Made list of points");
  points_it = points.begin();
  for(i = 0; i<points.size(); i++)
  {
    ROS_INFO("Point %d seen by:", points_it->first);
    inner_it = points_it->second.begin();
    for (j = 0; j<points_it->second.size();j++)
    {
      ROS_INFO("Kf %d at index %d",inner_it->first, inner_it->second);
      inner_it++;
    }
    points_it++;
  }

  /* End of printing */
  boris_drone::BundleMsg::Ptr msg(new boris_drone::BundleMsg);

  msg->num_cameras = ncam;
  msg->num_points  = npt;
  msg->num_observations = nobs;
  msg->observations.resize(nobs);
  msg->points.resize(npt);
  msg->points_ID.resize(npt);
  k = 0;
  std::map<int,int> this_point;

  points_it = points.begin();
  for (i = 0; i < npt; ++i)
  {
    int ptID   = points_it->first;
    this_point = points_it->second;
    for (inner_it = this_point.begin(); inner_it != this_point.end(); ++inner_it)
    {
      int kfID = inner_it->first; //This is the ID
      int local_idx = inner_it->second;
      msg->observations[k].kf_ID = kfID;
      msg->observations[k].pt_ID  = ptID;
      ROS_INFO("Observation %d: Cam index = %d; Point index = %d",k,kfID,ptID);
      msg->observations[k].x = keyframes[kfID]->img_points[local_idx].x;
      msg->observations[k].y = keyframes[kfID]->img_points[local_idx].y;
      k++;
    }
    msg->points_ID[i] = ptID;
    msg->points[i].x = cloud->points[pt_idxes[ptID]].x; //TODO: fix this
    msg->points[i].y = cloud->points[pt_idxes[ptID]].y;
    msg->points[i].z = cloud->points[pt_idxes[ptID]].z;
    points_it++;
  }
  msg->cameras.resize(ncam);
  msg->fixed_cams.resize(ncam);
  msg->keyframes_ID.resize(ncam);
  for (i = 0; i < ncam; ++i) {
    msg->cameras[i].x    = keyframes[kf_IDs[i]]->pose.x;
    msg->cameras[i].y    = keyframes[kf_IDs[i]]->pose.y;
    msg->cameras[i].z    = keyframes[kf_IDs[i]]->pose.z;
    msg->cameras[i].rotX = keyframes[kf_IDs[i]]->pose.rotX;
    msg->cameras[i].rotY = keyframes[kf_IDs[i]]->pose.rotY;
    msg->cameras[i].rotZ = keyframes[kf_IDs[i]]->pose.rotZ;
    msg->keyframes_ID[i] = kf_IDs[i];
    msg->fixed_cams[i]   = is_fixed[kf_IDs[i]];
  }
  ROS_INFO("About to publish to bundle_pub");
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

  PnP_pose.xvel    = 0.0;
  PnP_pose.yvel    = 0.0;
  PnP_pose.zvel    = 0.0;
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

void cloud_debug(pcl::PointCloud< pcl::PointXYZ >::ConstPtr cloud)
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
  pcl::PointXYZ pcl_point;
  matchDescriptors(this->descriptors, frame.descriptors, map_indices, frame_indices);
  if (map_indices.size() < threshold_lost)
    return -3;

  for (unsigned k = 0; k < map_indices.size(); k++)
  {
      pcl_point = this->cloud->points[map_indices[k]];
      cv::Point3f map_point(pcl_point.x,pcl_point.y,pcl_point.z);
      map_matching_points.push_back(map_point);
      frame_matching_points.push_back(frame.img_points[frame_indices[k]]);
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
  int npt, ncam, i, kf_ID, pt_ID;
  npt  = bundlePtr->num_points;
  ncam = bundlePtr->num_cameras;
  bool converged = bundlePtr->converged;
  ROS_INFO("Update Bundle");
  //if (!converged)
  //{
  //  ROS_INFO("Bundle adjustment did not converge. Not updating map");
  //  return;
  //}
  for (i = 0; i < npt; ++i)
  {
    pt_ID = bundlePtr->points_ID[i];
    pcl::PointXYZ new_point;
    new_point.x = bundlePtr->points[i].x;
    new_point.y = bundlePtr->points[i].y;
    new_point.z = bundlePtr->points[i].z;
    updatePoint(pt_ID,new_point);
  }
  ROS_INFO("Updated points, now cams:");
  for (i = 0; i < ncam; ++i) {
    kf_ID = bundlePtr->keyframes_ID[i];
    ROS_INFO("Updating keyframe %d",kf_ID);
    keyframes[kf_ID]->pose.x    = bundlePtr->cameras[i].x    ;
    keyframes[kf_ID]->pose.y    = bundlePtr->cameras[i].y    ;
    keyframes[kf_ID]->pose.z    = bundlePtr->cameras[i].z    ;
    keyframes[kf_ID]->pose.rotX = bundlePtr->cameras[i].rotX ;
    keyframes[kf_ID]->pose.rotY = bundlePtr->cameras[i].rotY ;
    keyframes[kf_ID]->pose.rotZ = bundlePtr->cameras[i].rotZ ;
    ROS_INFO("Done");
  }
}
