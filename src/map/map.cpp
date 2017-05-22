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
  reference_keyframe = new Keyframe(this, frame);
  this->keyframes.push_back(reference_keyframe);
  int nkeyframes = keyframes.size();
  ROS_INFO("Created new Keyframe. There are %d keyframes",nkeyframes);
  if (nkeyframes > 1)
  {
    matchKeyframes(*keyframes[nkeyframes-2], *keyframes[nkeyframes-1], true);
  }
}

bool Map::processFrame(const Frame& frame, boris_drone::Pose3D& PnP_pose)
{
  bool PnP_success = doPnP(frame,PnP_pose);
  if (PnP_success && abs(PnP_pose.z - frame.pose.z) > 0.8)  // treshold in cm
  {
    ROS_INFO("TRACKING LOST ! (PnP found bad symmetric clone?)");
    PnP_success = false;
  }
  if (!PnP_success)
  {
    if (keyframeNeeded(frame.pose))
    {
      newKeyframe(frame);
    }
  }
  else
  {
    ROS_INFO_THROTTLE(3,"published pnp_pose. pnp_pose is: x = %f, y = %f, z = %f",PnP_pose.x,PnP_pose.y,PnP_pose.z);
  }
  return PnP_success;
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
  this->reference_keyframe = new Keyframe(this,pose,current_frame);
  this->keyframes.push_back(reference_keyframe);
  ROS_INFO("Made new keyframe. There are %lu keyframes",this->keyframes.size());

}

bool Map::triangulate(cv::Point3d& pt_out, const cv::Point2d& pt1, const cv::Point2d& pt2,
                      const boris_drone::Pose3D& pose1, const boris_drone::Pose3D& pose2)
{
//in matlab this is triangulate4 lol
//http://www.iim.cs.tut.ac.jp/~kanatani/papers/sstriang.pdf
//(iterative method for higher order aproximation)
  /* get coordinates */
  cv::Mat cam2world1, cam2world2, origin1, origin2;
  cv::Mat T1, T2, K1, K2, cam0, cam1, F;
  cv::Mat pt1_h, pt2_h, u, P, x_hat, x1_hat, temp, dx, dx1;
  getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  getCameraPositionMatrices(pose2, cam2world2, origin2, true);

  T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin1.at<double>(0,0),
                                  0, 1, 0, -origin1.at<double>(1,0),
                                  0, 0, 1, -origin1.at<double>(2,0)
  );
  T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin2.at<double>(0,0),
                                  0, 1, 0, -origin2.at<double>(1,0),
                                  0, 0, 1, -origin2.at<double>(2,0)
  );
  K2 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                  0,     529.1, 182.2,
                                  0,     0,     1
  );
  K1 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                  0,     529.1, 182.2,
                                  0,     0,     1
  );

  //These are the camera projection matrices:
  // If p is a world point in 3D in the world coordinates,
  // then cam0*p are the image coordinates of the world point
  cam0 = K1*cam2world1.t()*T1;
  cam1 = K2*cam2world2.t()*T2;
  F = (cv::Mat_<double>(3, 3)); // Fundamental Matrix3x3
  getFundamentalMatrix(cam0,cam1,F);

  pt1_h = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);
  pt2_h = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);

  u = (cv::Mat_<double>(9,1) << F.at<double>(0,0),F.at<double>(0,1),F.at<double>(0,2),
                                F.at<double>(1,0),F.at<double>(1,1),F.at<double>(1,2),
                                F.at<double>(2,0),F.at<double>(2,1),F.at<double>(2,2));
  P = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,0);

  double x,y,x1,y1,f,x2,x12,y2,y12;

  x  = pt1.x; y  = pt1.y;
  x1 = pt2.x; y1 = pt2.y;
  f = 250;
  x2 = x*x; x12 = x1*x1;
  y2 = y*y; y12 = y1*y1;
  cv::Mat epsil = (cv::Mat_<double>(9,1) << x*x1,x*y1,x*f,y*x1,y*y1,y*f,x1*f,y1*f,f*f);
  cv::Mat V = (cv::Mat_<double>(9,9) <<
         x2+x12, x1*y1,  f*x1, x*y,    0,      0,    f*x, 0,   0,
         x1*y1,  x2+y12, f*y1, 0,      x*y,    0,    0,   f*x, 0,
         f*x1,   f*y1,   f*f,  0,      0,      0,    0,   0,   0,
         x*y,    0,      0,    y2+x12, x1*y1,  f*x1, f*y, 0,   0,
         0,      x*y,    0,    x1*y1,  y2+y12, f*y1, 0,   f*y, 0,
         0,      0,      0,    f*x1,   f*y1,   f*f,  0  , 0,   0,
         f*x,    0,      0,    f*y,    0,      0,    f*f, 0,   0,
         0,      f*x,    0,    0,      f*y,    0,    0,   f*f, 0,
         0,      0,      0,    0,      0,      0,    0,   0,   0
  );


  x_hat  = pt1_h;
  x1_hat = pt2_h;

  temp = u.t()*V*u;
  double den = temp.at<double>(0,0);
  int k = 0;
  while (k < 5)
  {
    k = k+1;
    temp = u.t()*epsil;
    dx  = P*F    *x1_hat*temp.at<double>(0,0)/den;
    dx1 = P*F.t()*x_hat *temp.at<double>(0,0)/den;
    x_hat  = pt1_h - dx;
    x1_hat = pt2_h - dx1;
  }
  cv::Point2d p1,p2;
  p1.x = x_hat.at<double>(0,0);
  p1.y = x_hat.at<double>(1,0);
  p2.x = x1_hat.at<double>(0,0);
  p2.y = x1_hat.at<double>(1,0);

  std::vector<cv::Point2d> cam0pnts;
  std::vector<cv::Point2d> cam1pnts;
  cam0pnts.push_back(p1);
  cam1pnts.push_back(p2);

  cv::Mat pnts3D(4,cam0pnts.size(),CV_64F);
  cv::triangulatePoints(cam0,cam1,cam0pnts,cam1pnts,pnts3D);

  pt_out.x = pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0);
  pt_out.y = pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0);
  pt_out.z = pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0);

  /*
  //Reproject to check accuracy:
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
  */
  return true;
}




void Map::doBundleAdjustment(Keyframe& kf1,
                             Keyframe& kf2,
                             std::vector<int> matching_indices_1,
                             std::vector<int> matching_indices_2,
                             bool fixed_poses,
                             std::vector<cv::Point3d>& points3D)
{
  /****** INPUT DATA *****************/
  ROS_INFO("Doing bundle adjustment");
  std::vector<std::vector<cv::Point2d> > pointsImg;
  std::vector<std::vector<int> > visibility;
  std::vector<cv::Mat> cameraMatrix, distCoeffs, R, T;
  int NPOINTS = matching_indices_1.size(); // number of 3d points
  int NCAMS   = 2; // number of cameras (viewpoints)

//  points3D.resize(NPOINTS);

  // fill image projections
  pointsImg.resize(NCAMS);
  visibility.resize(NCAMS);
  distCoeffs.resize(NCAMS);
  cameraMatrix.resize(NCAMS);
  R.resize(NCAMS);
  T.resize(NCAMS);
  for (int i=0; i<NCAMS; i++)
  {
    pointsImg[i].resize(NPOINTS);
    visibility[i].resize(NPOINTS);
    distCoeffs[i]   = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    cameraMatrix[i] = (cv::Mat_<double>(3, 3) << Read::focal_length_x(), 0                     , Read::img_center_x(),
                                                 0,                      Read::focal_length_y(), Read::img_center_y(),
                                                 0,                      0,                      1                   );
  }
  for (int i=0; i<NPOINTS; i++)
  {
    pointsImg[0][i] = kf1.unmapped_imgPoints[matching_indices_1[i]];
    pointsImg[1][i] = kf2.unmapped_imgPoints[matching_indices_2[i]];
    visibility[0][i] = 1;
    visibility[1][i] = 1;
  }
  cv::Mat temp1, temp2;

  getCameraPositionMatrices(kf1.pose, temp1, T[0], true);
  getCameraPositionMatrices(kf2.pose, temp2, T[1], true);
  R[0] = temp1.t();
  R[1] = temp2.t();
  /***********************************/

  cout << "before" << endl;
  cout << "R0 = "<< endl << " "  << R[0] << endl << endl;
  cout << "R1 = "<< endl << " "  << R[1] << endl << endl;
  cout << "T0 = "<< endl << " "  << T[0] << endl << endl;
  cout << "T1 = "<< endl << " "  << T[1] << endl << endl;

  /****** RUN BUNDLE ADJUSTMENT ******/
  cvsba::Sba sba;
  cvsba::Sba::Params params;
  if (fixed_poses)
  {
    params.type = cvsba::Sba::STRUCTURE;
  }
  else
  {
    params.type = cvsba::Sba::MOTIONSTRUCTURE;
  }
  params.type = cvsba::Sba::MOTIONSTRUCTURE;
  params.fixedIntrinsics = 5;
  params.fixedDistortion = 5;
  params.verbose = true;
  sba.setParams(params);
  sba.run(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
  cout << "after" << endl;
  cout << "R0 = "<< endl << " "  << R[0] << endl << endl;
  cout << "R1 = "<< endl << " "  << R[1] << endl << endl;
  cout << "T0 = "<< endl << " "  << T[0] << endl << endl;
  cout << "T1 = "<< endl << " "  << T[1] << endl << endl;
  /***********************************/
  //cv::LevMarqSparse::bundleAdjust(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
}

void Map::doBundleAdjustment2(Keyframe& kf1,
                              Keyframe& kf2,
                              std::vector<int> matching_indices_1,
                              std::vector<int> matching_indices_2,
                              bool fixed_poses,
                              std::vector<cv::Point3d>& points3D)
{
  boris_drone::BundleMsg::Ptr msg(new boris_drone::BundleMsg);
  int npt = matching_indices_1.size();
  msg->num_cameras = 2;
  msg->num_points  = npt;
  msg->num_observations = 2*npt;
  msg->observations.resize(2*npt);
  msg->points.resize(npt);
  for (int i = 0; i < npt; ++i) {
    msg->observations[2*i+0].camera_index = 0;
    msg->observations[2*i+0].point_index  = i;
    msg->observations[2*i+0].x = kf1.unmapped_imgPoints[matching_indices_1[i]].x;
    msg->observations[2*i+0].y = kf1.unmapped_imgPoints[matching_indices_1[i]].y;
    msg->observations[2*i+1].camera_index = 1;
    msg->observations[2*i+1].point_index  = i;
    msg->observations[2*i+1].x = kf2.unmapped_imgPoints[matching_indices_2[i]].x;
    msg->observations[2*i+1].y = kf2.unmapped_imgPoints[matching_indices_2[i]].y;

    msg->points[i].x = points3D[i].x;
    msg->points[i].y = points3D[i].y;
    msg->points[i].z = points3D[i].z;
  }
  msg->cameras.resize(2);
//TODO: make it general
  for (int i = 0; i < 2; ++i) {
    msg->cameras[i].rotX = -PI/2;
    msg->cameras[i].rotY = 0.0;
    msg->cameras[i].rotZ = -PI/2;
    msg->cameras[i].x     = 0.0;
    msg->cameras[i].y     = 0.0;
  }
  msg->cameras[0].z = 0.0;
  msg->cameras[1].z = 0.645;
  bundle_pub.publish(*msg);
}


bool Map::doPnP(const Frame& current_frame, boris_drone::Pose3D& PnP_pose)
{
  std::vector<cv::Point3f> inliers_map_matching_points;
  std::vector<cv::Point2f> inliers_frame_matching_points;

  bool match_success = matchWithFrame(current_frame, inliers_map_matching_points, inliers_frame_matching_points);

  cv::Mat_<double> tcam, cam2world, world2drone, distCoeffs;

  distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);
  // solve with P3P

  if (!match_success)
  {
    return false;
  }

  // solve with PnP n>3
  cv::solvePnP(inliers_map_matching_points, inliers_frame_matching_points,
               this->camera_matrix_K, distCoeffs, rvec, tvec, true, CV_EPNP);

  cv::Rodrigues(rvec, cam2world);

  if (fabs(determinant(cam2world)) - 1 > 1e-07)
  {
    ROS_DEBUG("TRACKING LOST ! (Determinant of rotation matrix)");
    return false;
  }

  //front camera:
  cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI/2, 0, -PI / 2);
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
  return true;
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
  if (keyframes.size() == 0)
  {
    return true;
  }
  else if ( (keyframes.size() == 1) && (pose.z > keyframes[0]->pose.z + 0.5) )
  {
    return true;
  }
  else if (keyframes.size() >= 2)
  {
    double sqDist = getSqDist(pose, keyframes[keyframes.size()-1]->pose);
    return (sqDist > 1);
  }
  return false;
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%lu] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}

bool Map::matchWithFrame(const Frame& frame, std::vector<cv::Point3f>& inliers_map_matching_points,
                                             std::vector<cv::Point2f>& inliers_frame_matching_points)
{
  if (frame.descriptors.rows == 0)
  {
    ROS_INFO("Frame is empty");
    return false;
  }
  if (this->descriptors.rows == 0)
  {
    ROS_INFO_THROTTLE(3,"Map is empty");
    return false;
  }

  std::vector<cv::Point3f> map_matching_points;
  std::vector<cv::Point2f> frame_matching_points;

  std::vector<cv::DMatch> simple_matches;
  matcher.match(frame.descriptors, this->descriptors, simple_matches);

  // threshold test
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      pcl::PointXYZRGBSIFT pcl_point = this->cloud->points[simple_matches[k].trainIdx];
      cv::Point3f map_point(pcl_point.x,pcl_point.y,pcl_point.z);

      map_matching_points.push_back(map_point);
      frame_matching_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
    }
  }

  if (map_matching_points.size() < threshold_lost)
  {
    ROS_INFO("Tracking lost! not enough matching points (%lu)", map_matching_points.size());
    return false;
  }
  cv::Mat distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);
  std::vector<int> inliers;
  cv::solvePnPRansac(map_matching_points, frame_matching_points, this->camera_matrix_K, distCoeffs, rvec, tvec,
                     true, 2500, 2, 2 * threshold_new_keyframe, inliers, CV_P3P);  // or: CV_EPNP and CV_ITERATIVE
  if (inliers.size() < threshold_lost)
  {
    ROS_INFO("Tracking lost! not enough inliers (%lu inliers, %lu matching points)",
                inliers.size(),map_matching_points.size());
    return false;
  }

  for (int j = 0; j < inliers.size(); j++)
  {
    int i = inliers[j];

    inliers_map_matching_points.push_back(map_matching_points[i]);
    inliers_frame_matching_points.push_back(frame_matching_points[i]);
  }
  return true;
}


void Map::matchKeyframes(Keyframe& kf1, Keyframe& kf2, bool fixed_poses)
{
  if (kf1.descriptors.rows == 0 || kf2.descriptors.rows == 0)
    return;

  std::vector<cv::DMatch> simple_matches;
  matcher.match(kf1.descriptors, kf2.descriptors, simple_matches);

  // threshold test
  std::vector<int> matching_indices_1;
  std::vector<int> matching_indices_2;
  for (unsigned k = 0; k < simple_matches.size(); k++)
  {
    if (simple_matches[k].distance < DIST_THRESHOLD)
    {
      matching_indices_1.push_back(simple_matches[k].queryIdx);
      matching_indices_2.push_back(simple_matches[k].trainIdx);
    }
  }
  ROS_INFO("Matching keyframes: there are %lu,%lu matching points",
           matching_indices_1.size(),matching_indices_2.size());

  std::vector<cv::Point3d> points3D;
  for (int i = 0; i<matching_indices_1.size();i++)
  {
    cv::Point3d pt3d;
    triangulate(pt3d, kf1.unmapped_imgPoints[matching_indices_1[i]],
                       kf2.unmapped_imgPoints[matching_indices_2[i]],
                       kf1.pose, kf2.pose);
    points3D.push_back(pt3d);
  }
  doBundleAdjustment2(kf1, kf2, matching_indices_1, matching_indices_2, fixed_poses, points3D);

  /* TODO: put code in keyframe
  kf1.designPointsAsMapped(matching_indices_1,cloud_indices);
  kf2.designPointsAsMapped(matching_indices_2,cloud_indices);
  */

  //add points to cloud and to mapped points of both keyframes
  for (int i=0; i<points3D.size(); i++)
  {
    pcl::PointXYZRGBSIFT new_point;
    new_point.x = points3D[i].x;
    new_point.y = points3D[i].y;
    new_point.z = points3D[i].z;

    int new_pt_idx = cloud->points.size();
    kf1.points.push_back(new_pt_idx);
    kf2.points.push_back(new_pt_idx);
    cloud->points.push_back(new_point);

    cv::Range row = cv::Range(matching_indices_1[i],matching_indices_1[i]+1);

    this->descriptors.push_back(kf1.descriptors(row,cv::Range::all()));

    kf1.mapped_imgPoints.push_back(kf1.unmapped_imgPoints[matching_indices_1[i]]);
    kf2.mapped_imgPoints.push_back(kf2.unmapped_imgPoints[matching_indices_2[i]]);
  }

  //remove points from unmatched points of both keyframes
  //remove highest indices first, so each index is still relevant
  std::sort(matching_indices_1.begin(), matching_indices_1.end());
  std::sort(matching_indices_2.begin(), matching_indices_2.end());
  for (int i = matching_indices_1.size()-1; i>=0; i--)
  {
    kf1.unmapped_imgPoints.erase(kf1.unmapped_imgPoints.begin() + matching_indices_1[i]);
    kf2.unmapped_imgPoints.erase(kf2.unmapped_imgPoints.begin() + matching_indices_2[i]);
  }
  ROS_INFO("finished matching keyframes. Map now has %lu points",cloud->points.size());
  ROS_INFO("descriptor check: %d rows, %d cols", descriptors.rows, descriptors.cols);
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
