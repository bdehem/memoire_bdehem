//in header:


void matchKeyframes(Keyframe& kf1, Keyframe& kf2, bool fixed_poses);



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
    triangulate(pt3d, kf1.unmapped_img_points[matching_indices_1[i]],
                      kf2.unmapped_img_points[matching_indices_2[i]],
                      kf1.pose, kf2.pose);
    points3D.push_back(pt3d);
  }
  std::vector<Keyframe*> kfs;
  kfs.push_back(&kf1);
  kfs.push_back(&kf2);
  doBundleAdjustment(kfs, matching_indices_1, matching_indices_2, fixed_poses, points3D);

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

    int new_ptIDx = cloud->points.size();
    kf1.points.push_back(new_ptIDx);
    kf2.points.push_back(new_ptIDx);
    cloud->points.push_back(new_point);

    cv::Range row = cv::Range(matching_indices_1[i],matching_indices_1[i]+1);

    this->descriptors.push_back(kf1.descriptors(row,cv::Range::all()));

    kf1.mapped_img_points.push_back(kf1.unmapped_img_points[matching_indices_1[i]]);
    kf2.mapped_img_points.push_back(kf2.unmapped_img_points[matching_indices_2[i]]);
  }

  //remove points from unmatched points of both keyframes
  //remove highest indices first, so each index is still relevant
  std::sort(matching_indices_1.begin(), matching_indices_1.end());
  std::sort(matching_indices_2.begin(), matching_indices_2.end());
  for (int i = matching_indices_1.size()-1; i>=0; i--)
  {
    kf1.unmapped_img_points.erase(kf1.unmapped_img_points.begin() + matching_indices_1[i]);
    kf2.unmapped_img_points.erase(kf2.unmapped_img_points.begin() + matching_indices_2[i]);
  }
  ROS_INFO("finished matching keyframes. Map now has %lu points",cloud->points.size());
  ROS_INFO("descriptor check: %d rows, %d cols", descriptors.rows, descriptors.cols);
}



//replaced by doBundleAdjustment

  void doBundleAdjustment(Keyframe& kf1,
                          Keyframe& kf2,
                          std::vector<int> matching_indices_1,
                          std::vector<int> matching_indices_2,
                          bool fixed_poses,
                          std::vector<cv::Point3d>& points3D);
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
    pointsImg[0][i] = kf1.unmapped_img_points[matching_indices_1[i]];
    pointsImg[1][i] = kf2.unmapped_img_points[matching_indices_2[i]];
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





//just before generalization th n kfs:

void Map::doBundleAdjustment(std::vector<Keyframe*> kfs,
                              std::vector<int> matching_indices_1,
                              std::vector<int> matching_indices_2,
                              bool fixed_poses,
                              std::vector<cv::Point3d>& points3D)
{
  int ncam, npt, nobs, npt_cam, i, j, ptIdx, pos;
  std::set<int> ba_points;
  std::set<int>::iterator ba_pts_it;
  std::vector< std::vector<int> > kfs_point; //kfs (inner vec) seeing point (outer vec)
  std::pair<std::set<int>::iterator,bool> ouputOfInsert;

  ncam = kfs.size();
  for(i = 0; i<ncam; i++)
  {
    npt_cam = kfs[i]->npts;
    for(j = 0; j < npt_cam; j++)
    {
      if (kfs[i]->point_is_mapped[j])
      {
        ptIdx = kfs[i]->points[j];
        ouputOfInsert = ba_points.insert(ptIdx);
        pos = std::distance(ba_points.begin(), ouputOfInsert.first);
        if (ouputOfInsert.second==false) // ptIdx was already in ba_points
        {
          kfs_point[pos].push_back(i);
        }
        else
        {
          std::vector<int> kf_thispoint(1,i); //vector containing 1 element: i
          kfs_point.insert(kfs_point.begin() + pos, kf_thispoint);
        }
      }
    }
  }
  //Remove points that are only seen once and count observations
  i = 0;
  nobs = 0;
  for (ba_pts_it = ba_points.begin(); ba_pts_it != ba_points.end(); ba_pts_it++)
  {
    if(kfs_point[i].size() < 2)
    {
      ba_points.erase(ba_pts_it);
      kfs_point.erase(kfs_point.begin() + i);
    }
    else
    {
      nobs += kfs_point[i].size();
    }
    i++;
  }


  /*
  print set info
  ROS_INFO("Preparing bundle. N_ba_points = %lu", ba_points.size());
  ROS_INFO("check: %lu = %lu", ba_points.size(), kfs_point.size());
  std::set<int>::iterator ba_pts_it = ba_points.begin();
  for(i = 0; i<ba_points.size(); i++)
  {
    ROS_INFO("Point %d seen by:",*it);
    for (j = 0; j<kfs_point[i].size();j++)
    {
      ROS_INFO("%d",kfs_point[i][j]);
    }
    it++;
  }

  */

  boris_drone::BundleMsg::Ptr msg(new boris_drone::BundleMsg);
  npt = ba_points.size();
  msg->num_cameras = ncam;
  msg->num_points  = npt;
  msg->num_observations = nobs;
  msg->observations.resize(nobs);
  msg->points.resize(npt);
  for (int i = 0; i < npt; ++i) {
    msg->observations[2*i+0].camera_index = 0;
    msg->observations[2*i+0].point_index  = i;
    msg->observations[2*i+0].x = kfs[0]->unmapped_img_points[matching_indices_1[i]].x;
    msg->observations[2*i+0].y = kfs[0]->unmapped_img_points[matching_indices_1[i]].y;
    msg->observations[2*i+1].camera_index = 1;
    msg->observations[2*i+1].point_index  = i;
    msg->observations[2*i+1].x = kfs[1]->unmapped_img_points[matching_indices_2[i]].x;
    msg->observations[2*i+1].y = kfs[1]->unmapped_img_points[matching_indices_2[i]].y;

    msg->points[i].x = points3D[i].x;
    msg->points[i].y = points3D[i].y;
    msg->points[i].z = points3D[i].z;
  }
  msg->cameras.resize(2);
  for (int i = 0; i < 2; ++i) {
    tf::Matrix3x3 drone2world, cam2drone, cam2world;
    double roll, pitch, yaw;
    drone2world.setRPY(kfs[i]->pose.rotX,kfs[i]->pose.rotY,kfs[i]->pose.rotZ);
    cam2drone.setRPY(-PI/2, 0, -PI/2);
    cam2world = drone2world*cam2drone;
    cam2world.getRPY(roll, pitch, yaw);
    cout << "=== Camera number " << i << " ==="<< std::endl;
    cout << "drone2world RPY" << std::endl;
    cout <<  kfs[i]->pose.rotX << ";  " << kfs[i]->pose.rotY << ";  " << kfs[i]->pose.rotZ  << std::endl;
    cout << "cam2drone RPY" << std::endl;
    cout <<  -PI/2 << ";  " << 0 << ";  " << -PI/2  << std::endl;
    cout << "RPY" << std::endl;
    cout <<  roll << ";  " << pitch << ";  " << yaw  << std::endl;
    msg->cameras[i].rotX = roll;
    msg->cameras[i].rotY = pitch;
    msg->cameras[i].rotZ = yaw;
    msg->cameras[i].x = kfs[i]->pose.x;
    msg->cameras[i].y = kfs[i]->pose.y;
    msg->cameras[i].z = kfs[i]->pose.z;

  }
  bundle_pub.publish(*msg);
}




bool Map::keyframeNeeded(boris_drone::Pose3D pose)
{
  if (keyframes.size() == 0)
    return true;
  else if ( (keyframes.size() == 1) &&
            ((pose.z > keyframes[0]->pose.z + 0.5)|| (pose.y > keyframes[0]->pose.y + 0.5)) )
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


/*!
 * This method creates a new keyframe, sets it as the reference keyframe, and pushes it on the keyframe list
 */
void newReferenceKeyframe(const Frame& current_frame, boris_drone::Pose3D PnP_pose, bool PnP_success);


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



//part of dobundleadjustment to collect indices

  Keyframe* kf;
  std::map<int, Keyframe*>::iterator iter;
  //for (iter = keyframes.begin(); iter != keyframes.end(); ++iter)
  for (i = 0; i < ncam; ++i)
  {
    kfID = kfIDs[i];
    kf    = keyframes[kfID];
    npt_cam = kf->npts;
    for(j = 0; j < npt_cam; j++)
    {
      if (kf->point_is_mapped[j]) //Only take points that are mapped
      {
        ptIdx = kf->points[j]; //This is the index of the point in the map
        ouputOfInsert = ba_points.insert(ptIdx);
        //pos is the position where the point was inserted,
        //or the position where it already was
        pos = std::distance(ba_points.begin(), ouputOfInsert.first);
        if (ouputOfInsert.second==false) // ptIdx was already in ba_points
        {
          kfs_point_local[pos].push_back(i); //Add this kf's ID to the kfs seeing this pt
          idx_point_in_kf_local[pos].push_back(j); //idx of the pt in the kf
        }
        else //ptIdx wasn't in ba_points before the insert
        {
          std::vector<int> kf_thispoint(1,i); //vector containing 1 element: i
          std::vector<int> idx_thispt_in_thiskf(1,j); //index of this point in this kf
          kfs_point_local.insert(kfs_point_local.begin() + pos, kf_thispoint);
          idx_point_in_kf_local.insert(idx_point_in_kf_local.begin() + pos, idx_thispt_in_thiskf);
        }
      }
    }
  }

  //Remove points that are only seen once and count observations
  i = kfs_point_local.size();
  nobs = 0;
  for (ba_pts_it = ba_points.end(); ba_pts_it != ba_points.begin(); )
  {
    i--;
    if(kfs_point_local[i].size() < 2)
    {
      //ROS_INFO("About to erase point %d",i);
      ba_points.erase(ba_pts_it--);
      kfs_point_local.erase(kfs_point_local.begin() + i);
      idx_point_in_kf_local.erase(idx_point_in_kf_local.begin() + i);
      //ROS_INFO("Erased point %d",i);
    }
    else
    {
      ba_pts_it--;
      //ROS_INFO("Not erasing point %d. It was seen by %lu kfs",i,kfs_point_local[i].size());
      nobs += kfs_point_local[i].size();
      //ROS_INFO("Total nobs = %d",nobs);
    }
  }
