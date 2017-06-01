#include <boris_drone/map/map.h>

int Map::point_ID_counter = 0;

Map::Map() {}

Map::Map(ros::NodeHandle* nh) : cloud(new pcl::PointCloud< pcl::PointXYZ >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF

  nh = nh;
  bundle_channel = nh->resolveName("bundle");
  bundle_pub     = nh->advertise<boris_drone::BundleMsg>(bundle_channel, 1);

  benchmark_channel = nh->resolveName("benchmark");
  benchmark_pub     = nh->advertise<boris_drone::BenchmarkInfoMsg>(benchmark_channel, 1);


  ros::param::get("~2D_noise", use_2D_noise);
  ros::param::get("~3D_noise", use_3D_noise);
  ros::param::get("~threshold_kf_match", threshold_kf_match);
  ROS_INFO("init map, thresh = %f",threshold_kf_match);
  // define some threshold used later
  // better if defined in the launch file
  n_keyframes   = 0;
  frame_counter = 0;
  is_adjusting_bundle     = false;
  second_keyframe_pending = false;
  camera = Camera(true);
  // get camera parameters in launch file
  if (!Read::CamMatrixParams("cam_matrix"))
    ROS_ERROR("cam_matrix not properly transmitted");
  if (!Read::ImgSizeParams("img_size"))
    ROS_ERROR("img_size not properly transmitted");

  // initialize empty opencv vectors
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);

  ROS_DEBUG("map initialized");
}

Map::~Map() {reset();}

void Map::reset()
{
  std::map<int, Keyframe*>::iterator iter;
  for (iter = keyframes.begin(); iter != keyframes.end(); ++iter)
    delete iter->second;
  keyframes.clear();
  cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);
  tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  rvec = cv::Mat::zeros(3, 1, CV_64FC1);
}

bool Map::isInitialized(){  return (keyframes.size() > 3);}

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
    ROS_INFO("Trying to update point %d, but it does not exist",pt_ID);
    return;
  }
  cloud->points[pt_idxes[pt_ID]] = new_point;
}

void Map::removePoint(int ptID)
{
  std::map<int,int>::iterator idxes_it = pt_idxes.find(ptID);
  if (idxes_it == pt_idxes.end())
  {
    ROS_INFO("Trying to remove point %d, but it does not exist",ptID);
    return;
  }
  int index = idxes_it->second;
  std::set<int> kfs = keyframes_seeing_point[ptID];
  std::set<int>::iterator kf_it;
  for (kf_it = kfs.begin();kf_it!=kfs.end();++kf_it)
  {
    keyframes[*kf_it]->removePoint(ptID);
    points_seen_by_keyframe[*kf_it].erase(ptID);
  }
  keyframes_seeing_point.erase(ptID);
  pt_idxes.erase(ptID);
  pt_IDs.erase(pt_IDs.begin()+index);
  cloud->erase(cloud->begin()+index);
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
  //Decrement all indices that were after the one we removed
  for(idxes_it = pt_idxes.begin();idxes_it!=pt_idxes.end();++idxes_it)
    if (idxes_it->second > index)
      idxes_it->second--;
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

  match(kf0, kf1, points3D, idx_kf0, idx_kf1, match_ID, point_is_new, next_point_ID, threshold_kf_match);
  for (i = 0; i<match_ID.size(); i++)
  {
    if (point_is_new[i])
    {
      pt_ID = addPoint(points3D[i], kf0.descriptors.rowRange(idx_kf0[i],idx_kf0[i]+1));
      setPointAsSeen(pt_ID, kf0.ID);
      setPointAsSeen(pt_ID, kf1.ID);
    }
    else if(match_ID[i] >= 0)
    {
      pt_ID = match_ID[i];
      setPointAsSeen(pt_ID, kf0.ID);
      setPointAsSeen(pt_ID, kf1.ID);
    }
  }
}

void Map::newPairOfKeyframes(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose)
{
  Frame middle_frame = queue_of_frames.front();
  if ((frame.img_points.size() <= 10)||(middle_frame.img_points.size() <= 10))
  {
    ROS_INFO("I want to create a new keyframe, but current frame only has %lu points",frame.img_points.size());
    return;
  }

  Keyframe* middle_keyframe = new Keyframe(cloud,middle_frame,&camera);
  last_new_keyframe = ros::Time::now();
  n_keyframes++;
  if (n_keyframes>n_kf_for_ba)
    start_kf_for_ba++;
  else
    start_kf_for_ba = keyframes.begin();

  reference_keyframe = use_pose ? new Keyframe(cloud,frame,&camera,pose) : new Keyframe(cloud,frame,&camera);
  last_new_keyframe = ros::Time::now();
  n_keyframes++;
  if (n_keyframes>n_kf_for_ba)
    start_kf_for_ba++;
  else
    start_kf_for_ba = keyframes.begin();

  std::vector<int> keyframes_to_adjust;
  std::map<int,Keyframe*>::iterator it;
  for (it = start_kf_for_ba; it!=keyframes.end(); it++)
  {
    matchKeyframes(*reference_keyframe, *(it->second));
    matchKeyframes(*middle_keyframe, *(it->second));
    keyframes_to_adjust.push_back(it->first); //add all kfs
  }
  matchKeyframes(*reference_keyframe, *middle_keyframe);
  keyframes_to_adjust.push_back(reference_keyframe->ID);
  keyframes_to_adjust.push_back(middle_keyframe->ID);
  keyframes[reference_keyframe->ID] = reference_keyframe;
  keyframes[middle_keyframe->ID] = middle_keyframe;

  ROS_INFO("\t Map now has %lu points",this->cloud->points.size());
  ROS_INFO("\t check: %d = %lu", this->descriptors.rows, this->cloud->points.size());
  doBundleAdjustment(keyframes_to_adjust,true);
}



void Map::newPairOfKeyframes2(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose)
{
  Frame middle_frame = queue_of_frames.front();
  newKeyframe(middle_frame, pose, false);
  second_keyframe_pending = true;
}

void Map::newKeyframe(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose)
{
  if (frame.img_points.size() <= 10)
  {
    ROS_INFO("I want to create a new keyframe, but current frame only has %lu points",frame.img_points.size());
    return;
  }
  reference_keyframe = use_pose ? new Keyframe(cloud,frame,&camera,pose) : new Keyframe(cloud,frame,&camera);

  last_new_keyframe = ros::Time::now();
  n_keyframes++;
  if (n_keyframes>n_kf_for_ba)
    start_kf_for_ba++;
  else
    start_kf_for_ba = keyframes.begin();
  if (keyframes.empty())
  {
    keyframes[reference_keyframe->ID] = reference_keyframe;
    return;
  }
  //int n_keyframes = keyframes.size();
  int n_keyframes_to_ignore = std::min((int)keyframes.size(), 2);
  std::vector<int> keyframes_to_adjust;
  std::map<int,Keyframe*>::iterator it;
  for (it = start_kf_for_ba; it!=keyframes.end(); it++)
  {
    matchKeyframes(*reference_keyframe, *(it->second)); //Sould be better but results are not great
    keyframes_to_adjust.push_back(it->first); //add all kfs
  }
  //matchKeyframes(*reference_keyframe, *old);
  keyframes_to_adjust.push_back(reference_keyframe->ID);
  keyframes[reference_keyframe->ID] = reference_keyframe;

  ROS_INFO("\t Map now has %lu points",this->cloud->points.size());
  ROS_INFO("\t check: %d = %lu", this->descriptors.rows, this->cloud->points.size());
  doBundleAdjustment(keyframes_to_adjust,true);
}


bool Map::processFrame(Frame& frame, boris_drone::Pose3D& PnP_pose, bool manual_pose_received)
{
  int n_inliers;
  int PnP_result = doPnP(frame, PnP_pose, n_inliers);
  int n_keyframes = keyframes.size();
  if (keyframeNeeded(manual_pose_received, n_inliers)&&!isInitialized())
  {
    if (use_2D_noise)
    {
      frame.pose.x    += 0.20*(double)(n_keyframes==1);
      frame.pose.y    += 0.20*(double)(n_keyframes==2);
      frame.pose.rotZ += (PI/10)*(double)(n_keyframes==3);
    }
    if (use_3D_noise)
    {
      frame.pose.z    += 0.20*(double)(n_keyframes==1);
      frame.pose.rotX += (PI/10)*(double)(n_keyframes==2);
      frame.pose.rotY += (PI/10)*(double)(n_keyframes==3);
    }
    newKeyframe(frame, PnP_pose, (!manual_pose_received)&&(PnP_result==1));
  }
  else if (isInitialized())
  {
    if (frame_counter % 5 == 0)
      queue_of_frames.push_back(frame);
    if (frame_counter % 10 == 0)
      queue_of_frames.pop_front();
    frame_counter++;
    if (keyframeNeeded(manual_pose_received, n_inliers))
    {
      newPairOfKeyframes2(frame, PnP_pose, (!manual_pose_received)&&(PnP_result==1));
      queue_of_frames.clear();
      frame_counter = 0;
    }
  }
  switch(PnP_result){
    case 1  : //PnP successful
      ROS_INFO_THROTTLE(4,"(%3d inliers) PnP_pose is: x = % 4.3f, rotX = % 4.3f", n_inliers, PnP_pose.x, PnP_pose.rotX);
      ROS_INFO_THROTTLE(4,"                         : y = % 4.3f, rotY = % 4.3f", PnP_pose.y, PnP_pose.rotY);
      ROS_INFO_THROTTLE(4,"                         : z = % 4.3f, rotZ = % 4.3f", PnP_pose.z, PnP_pose.rotZ);
      ROS_INFO_THROTTLE(4,"                                                    ");
      return true;
    case -1  : //Empty frame
      ROS_INFO_THROTTLE(4,"Frame is empty");
      return false;
    case -2  : //Empty map
      ROS_INFO_THROTTLE(4,"Map is empty");
      return false;
    case -3  :
      ROS_INFO_THROTTLE(4,"Tracking lost! not enough matching points");
      return false;
    case -4  :
      ROS_INFO_THROTTLE(4,"Tracking lost! not enough inliers");
      return false;
    case -5 :
      ROS_INFO_THROTTLE(4,"TRACKING LOST ! (Determinant of rotation matrix)");
      return false;
    case -6 :
      ROS_INFO_THROTTLE(4,"TRACKING LOST ! (PnP found bad symmetric clone?)");
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
      //ROS_INFO("erasing point %d", *it1);
      point_set.erase(it1++);
    }
    else
    {
      std::map<int,int> this_point;
      for (it2 = intersection.begin();it2!=intersection.end();++it2)
      {
        int local_index = keyframes[*it2]->point_idx[*it1];
        //ROS_INFO("in pointseenbykeyframes. pointID = %d, keyframeID = %d, local idx = %d",*it1,*it2,local_index);
        this_point[*it2] = keyframes[*it2]->point_idx[*it1];
      }
      points[*it1] = this_point;
      it1++;
      nobs += intersection.size();
    }
  }
  return nobs;
}

int Map::doPnP(const Frame& current_frame, boris_drone::Pose3D& PnP_pose, int& n_inliers)
{
  std::vector<cv::Point3f> inliers_map_matching_points;
  std::vector<cv::Point2f> inliers_frame_matching_points;
  int result = matchWithFrame(current_frame, inliers_map_matching_points, inliers_frame_matching_points);
  n_inliers = inliers_map_matching_points.size();
  if (result < 0)
    return result;

  cv::Mat_<double> tcam, world2cam, drone2world, distCoeffs;
  distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

  // solve with PnP n>3
  cv::solvePnP(inliers_map_matching_points, inliers_frame_matching_points,
               camera.get_K(), distCoeffs, rvec, tvec, true, CV_EPNP);

  cv::Rodrigues(rvec, world2cam);
  if (fabs(determinant(world2cam)) - 1 > 1e-07)
    return -5;

  //front camera:
  cv::Mat cam2drone = camera.get_R();

  tcam = -world2cam.t() * tvec;
  PnP_pose.x = tcam(0);
  PnP_pose.y = tcam(1);
  PnP_pose.z = tcam(2);

  if (abs(PnP_pose.z - current_frame.pose.z) > 0.8)
    return -6;

  cv::Mat_< double > cam2world = world2cam.t();
  cv::Mat_< double > drone2cam = cam2drone.t();
  drone2world = cam2world * drone2cam;

  tf::Matrix3x3(drone2world(0, 0), drone2world(0, 1), drone2world(0, 2),
  drone2world(1, 0), drone2world(1, 1), drone2world(1, 2),
  drone2world(2, 0), drone2world(2, 1), drone2world(2, 2))
  .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

  PnP_pose.xvel    = 0.0;
  PnP_pose.yvel    = 0.0;
  PnP_pose.zvel    = 0.0;
  PnP_pose.rotXvel = 0.0;
  PnP_pose.rotYvel = 0.0;
  PnP_pose.rotZvel = 0.0;

  PnP_pose.header.stamp = current_frame.pose.header.stamp;  // needed for rqt_plot
  return 1;
}


bool customLess(std::vector< int > a, std::vector< int > b)
{
  return a[1] > b[1];
}

bool Map::keyframeNeeded(bool manual_pose_received, int n_inliers)
{
  if ( isInitialized())     return false; //(for benchmark of initialization only)
  if (keyframes.size()==0)  return true;
  if (manual_pose_received) return true;
  if (!isInitialized())     return false;
  if (is_adjusting_bundle)  return false;
  if (second_keyframe_pending)
  {
    second_keyframe_pending = false;
    queue_of_frames.clear();
    frame_counter = 0;
    return true;
  }
  if (n_inliers>threshold_inliers_new_keyframe) return false;
  return (ros::Time::now() - last_new_keyframe) > ros::Duration(5.0);
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
  matchDescriptors(this->descriptors, frame.descriptors, map_indices, frame_indices,250.0);
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
  cv::solvePnPRansac(map_matching_points, frame_matching_points, camera.get_K(), distCoeffs, rvec, tvec,
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

void Map::doBundleAdjustment(std::vector<int> kf_IDs, bool is_first_pass)
{
  if (is_first_pass) is_adjusting_bundle = true;
  int ncam, npt, nobs, i, j, k;
  std::map<int,std::map<int,int> > points;
  std::map<int,std::map<int,int> >::iterator points_it;
  std::map<int,int>::iterator inner_it;
  nobs = getPointsSeenByKeyframes(kf_IDs, points);
  ncam = kf_IDs.size();
  npt  = points.size();

  /*  print a bunch of things */
  points_it = points.begin();
  for(i = 0; i<points.size(); i++)
  {
    //ROS_INFO("Point %d seen by:", points_it->first);
    inner_it = points_it->second.begin();
    for (j = 0; j<points_it->second.size();j++)
    {
      //ROS_INFO("Kf %d at index %d",inner_it->first, inner_it->second);
      inner_it++;
    }
    points_it++;
  }

  /* End of printing */
  boris_drone::BundleMsg::Ptr msg(new boris_drone::BundleMsg);

  msg->is_first_pass  = is_first_pass;
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
      //ROS_INFO("Observation %d: Cam index = %d; Point index = %d",k,kfID,ptID);
      msg->observations[k].x = keyframes[kfID]->img_points[local_idx].x;
      msg->observations[k].y = keyframes[kfID]->img_points[local_idx].y;
      k++;
    }
    msg->points_ID[i] = ptID;
    msg->points[i].x = cloud->points[pt_idxes[ptID]].x;
    msg->points[i].y = cloud->points[pt_idxes[ptID]].y;
    msg->points[i].z = cloud->points[pt_idxes[ptID]].z;
    points_it++;
  }
  msg->cameras.resize(ncam);
  msg->fixed_cams.resize(ncam);
  msg->keyframes_ID.resize(ncam);
  for (i = 0; i < ncam; ++i) {
    msg->cameras[i] = keyframes[kf_IDs[i]]->pose;
    msg->keyframes_ID[i] = kf_IDs[i];
  }
  if (points.size()==0)
    ROS_WARN("Warning: there are no matching points to do Bundle Adjustment");
  else
    bundle_pub.publish(*msg);
}




void Map::updateBundle(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  int npt, ncam, i, kf_ID, pt_ID;
  int n_kf_pt;
  double cost_thresh = 0.5;
  bool converged, is_first_pass, remove_point;
  std::vector<int> keyframes_to_adjust;
  converged     = bundlePtr->converged;
  is_first_pass = bundlePtr->is_first_pass;
  npt           = bundlePtr->num_points;
  ncam          = bundlePtr->num_cameras;
  for (i = 0; i < ncam; ++i) {
    kf_ID = bundlePtr->keyframes_ID[i];
    keyframes_to_adjust.push_back(kf_ID);
    ROS_INFO("Updating keyframe %d",kf_ID);
    keyframes[kf_ID]->pose = bundlePtr->cameras[i];
  }
  int pts_removed = 0;
  for (i = 0; i < npt; ++i)
  {
    pt_ID = bundlePtr->points_ID[i];
    n_kf_pt = keyframes_seeing_point[pt_ID].size();
    switch(n_kf_pt){
      case 2  :
        remove_point = (bundlePtr->cost_of_point[i]>1.0);
        break;
      case 3  :
        remove_point = (bundlePtr->cost_of_point[i]>1.0);
        break;
      case 4  :
        remove_point = (bundlePtr->cost_of_point[i]>5.0);
        break;
      default :
        remove_point = false;
    }
    if (remove_point)
    {
      removePoint(pt_ID);
      pts_removed++;
    }
    else
    {
      pcl::PointXYZ new_point;
      new_point.x = bundlePtr->points[i].x;
      new_point.y = bundlePtr->points[i].y;
      new_point.z = bundlePtr->points[i].z;
      updatePoint(pt_ID,new_point);
    }
  }
  ROS_INFO("Removed %d points",pts_removed);
  if (is_first_pass)
  {
    BA_times_pass1.push_back(bundlePtr->time_taken);
    doBundleAdjustment(keyframes_to_adjust,false);
  }
  else
  {
    BA_times_pass2.push_back(bundlePtr->time_taken);
    is_adjusting_bundle = false;
    last_new_keyframe   = ros::Time::now();
    publishBenchmarkInfo();
  }
}

void Map::publishBenchmarkInfo()
{
  boris_drone::BenchmarkInfoMsg::Ptr msg(new boris_drone::BenchmarkInfoMsg);

  msg->pts_map = pt_IDs.size();

  msg->keyframes.resize(keyframes.size());
  msg->keyframes_ID.resize(keyframes.size());
  msg->n_pts_keyframe.resize(keyframes.size());
  msg->n_mapped_pts_keyframe.resize(keyframes.size());
  std::map<int,Keyframe*>::iterator it;
  int i = 0;
  for (it = keyframes.begin(); it!=keyframes.end(); ++it)
  {
    msg->keyframes_ID[i]          = it->first;
    msg->keyframes[i]             = it->second->pose;
    msg->n_pts_keyframe[i]        = it->second->npts;
    msg->n_mapped_pts_keyframe[i] = it->second->n_mapped_pts;
    i++;
  }

  msg->BA_times_pass1.resize(BA_times_pass1.size());
  msg->BA_times_pass2.resize(BA_times_pass1.size());
  for (i = 0; i<BA_times_pass1.size(); ++i)
  {
    msg->BA_times_pass1[i] = BA_times_pass1[i];
    msg->BA_times_pass2[i] = BA_times_pass2[i];
  }
  benchmark_pub.publish(*msg);
}
