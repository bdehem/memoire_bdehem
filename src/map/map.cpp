#include <boris_drone/map/map.h>

Map::Map()
{
}

Map::Map(bool do_search, bool stop_if_lost, cv::Mat camera_matrix_K) : cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF


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

  target_altitude = -1.0;

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

void Map::triangulate(cv::Point3d& pt_out, const cv::Point2d& pt1, const cv::Point2d& pt2,
                      const boris_drone::Pose3D& pose1, const boris_drone::Pose3D& pose2)
{
  //http://geomalgorithms.com/a07-_distance.html

  /* get coordinates */
  cv::Mat world2cam1, world2cam2, origin1, origin2;
  getCameraPositionMatrices(pose1, world2cam1, origin1, true);
  getCameraPositionMatrices(pose2, world2cam2, origin2, true);

  /* compute point coordinates in calibrated image coordinates (=rays in camera coordinates) */
  cv::Mat ray1_cam = (cv::Mat_< double >(3, 1) << (pt1.x - Read::img_center_x()) / Read::focal_length_x(),
                                                  (pt1.y - Read::img_center_y()) / Read::focal_length_y(), 1);

  cv::Mat ray2_cam = (cv::Mat_< double >(3, 1) << (pt2.x - Read::img_center_x()) / Read::focal_length_x(),
                                                  (pt2.y - Read::img_center_y()) / Read::focal_length_y(), 1);
  /* convert to world coordinates */
  cv::Mat ray1 = world2cam1.t() * ray1_cam;
  cv::Mat ray2 = world2cam2.t() * ray2_cam;

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
}




/* TODO: optimal correction (see http://www.iim.cs.tut.ac.jp/~kanatani/papers/bmtriang.pdf)
void Map::triangulate(cv::Point2d pt1, cv::Point2d pt2, boris_drone::Pose3D pose1, boris_drone::Pose3D pose2)
{
  cv::Point3D camera_center_1(pose1.x,pose1.y,pose1.z);
  cv::Point3D camera_center_2(pose2.x,pose2.y,pose2.z);
  cv::Mat Rx(3, 3, DataType<float>::type); //rotation matrix around x axis
  cv::Mat Ry(3, 3, DataType<float>::type); //rotation matrix around y axis
  cv::Mat Rz(3, 3, DataType<float>::type); //rotation matrix around z axis
  cv::Mat R(3, 3, DataType<float>::type);  //total rotation matrix
  cv::Mat t(3, 3, DataType<float>::type);  //translation matrix (vector expressed as cross-product matrix)
  cv::Mat E(3, 3, DataType<float>::type);  //essential matrix
  cv::Mat F(3, 3, DataType<float>::type);  //fundamental matrix

  double roll  = pose1.rotX - pose2.rotX;
  double pitch = pose1.rotY - pose2.rotY;
  double yaw   = pose1.rotZ - pose2.rotZ;

  double dx = pose1.X - pose2.X;
  double dy = pose1.Y - pose2.Y;
  double dz = pose1.Z - pose2.Z;

  Rx = (cv::Mat_<double>(3, 3) <<  1, 0        ,  0         ,
                                   0, cos(roll), -sin(roll) ,
                                   0, sin(roll),  cos(roll) );

  Ry = (cv::Mat_<double>(3, 3) <<  cos(pitch), 0, sin(pitch) ,
                                   0,          1, 0          ,
                                  -sin(pitch), 0, cos(pitch) );

  Rz = (cv::Mat_<double>(3, 3) <<  cos(yaw), -sin(yaw), 0 ,
                                   sin(yaw),  cos(yaw), 0 ,
                                   0       ,  0       , 1 );

  R = Rx*Ry*Rz;

  t = (cv::Mat_<double>(3, 3) <<  0 , -dz,  dy ,
                                  dz,  0 , -dx ,
                                 -dy,  dx,  0  );
  t.at<double>(0,0) = pose1.x - pose2.x;
  t.at<double>(1,0) = pose1.y - pose2.y;
  t.at<double>(2,0) = pose1.z - pose2.z;

  E = R*t;

  F = camera_matrix_K.t()*E*camera_matrix_K;

  // see http://www.iim.cs.tut.ac.jp/~kanatani/papers/bmtriang.pdf to continue
}
*/


void Map::doBundleAdjustment(Keyframe& kf1,
                             Keyframe& kf2,
                             std::vector<int> matching_indices_1,
                             std::vector<int> matching_indices_2,
                             bool fixed_poses,
                             std::vector<cv::Point3d>& points3D)
{
  /****** INPUT DATA *****************/
  ROS_INFO("Doing bundel adjustment");
  std::vector<std::vector<cv::Point2d> > pointsImg;
  std::vector<std::vector<int> > visibility;
  std::vector<cv::Mat> cameraMatrix, distCoeffs, R, T;
  int NPOINTS = matching_indices_1.size(); // number of 3d points
  int NCAMS   = 2; // number of cameras (viewpoints)

  points3D.resize(NPOINTS);

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
    cv::Point3d point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    points3D[i] = point;
    pointsImg[0][i] = kf1.unmapped_imgPoints[matching_indices_1[i]];
    pointsImg[1][i] = kf2.unmapped_imgPoints[matching_indices_2[i]];
    visibility[0][i] = 1;
    visibility[1][i] = 1;
    ROS_INFO("n%d. img1=(%f,%f), on img2=(%f,%f)",i, pointsImg[0][i].x,pointsImg[0][i].y,pointsImg[1][i].x,pointsImg[1][i].y);
  }

  getCameraPositionMatrices(kf1.pose, R[0], T[0], true);
  getCameraPositionMatrices(kf2.pose, R[1], T[1], true);
  /***********************************/

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
  params.fixedIntrinsics = 5;
  params.fixedDistortion = 5;
  params.verbose = true;
  sba.setParams(params);
  //sba.run(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
  /***********************************/
  cv::LevMarqSparse::bundleAdjust(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
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
  else if (keyframes.size() == 1)
  {
    return (pose.z > keyframes[0]->pose.z + 0.5);
  }
  else
  {
    double sqDist = getSqDist(pose, keyframes[keyframes.size()-1]->pose);
    return (sqDist > 1);
  }
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


//TODO: remove descriptors for keyframe
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
  points3D.resize(matching_indices_1.size());
  for (int i = 0; i<matching_indices_1.size();i++)
  {
    triangulate(points3D[i], kf1.unmapped_imgPoints[matching_indices_1[i]],
                             kf2.unmapped_imgPoints[matching_indices_2[i]],
                             kf1.pose, kf2.pose);
  }
  doBundleAdjustment(kf1, kf2, matching_indices_1, matching_indices_2, fixed_poses, points3D);

  /* TODO: put code in keyframe
  kf1.designPointsAsMapped(matching_indices_1,cloud_indices);
  kf2.designPointsAsMapped(matching_indices_2,cloud_indices);
  */

  //add points to cloud and to mapped points of both keyframes
  for (int i=0; i<matching_indices_1.size();i++)
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
