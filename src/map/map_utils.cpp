

#include <boris_drone/map/map_utils.h>

void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold, int max_matches)
{
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> simple_matches;
  std::vector<int>::iterator it;
  int train_idx;
  matcher.match(descriptors1, descriptors2, simple_matches);
  int nmatch = simple_matches.size();
  std::sort(simple_matches.begin(), simple_matches.end());

  if((max_matches < 0)||(max_matches > nmatch))
    max_matches = nmatch;
  for (unsigned k = 0; k < max_matches; k++)
  {
    if ((simple_matches[k].distance > threshold))
      break;
    it = find(matching_indices_2.begin(),matching_indices_2.end(),simple_matches[k].trainIdx);
    if (it==matching_indices_2.end())
    {
      matching_indices_1.push_back(simple_matches[k].queryIdx);
      matching_indices_2.push_back(simple_matches[k].trainIdx);
    }
  }
}

double poseDistance(boris_drone::Pose3D pose0, boris_drone::Pose3D pose1)
{
  return sqrt((pose0.x-pose1.x)*(pose0.x-pose1.x) +
              (pose0.y-pose1.y)*(pose0.y-pose1.y) +
              (pose0.z-pose1.z)*(pose0.z-pose1.z));
}

void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  const std::vector<int>& ptIDs1, const std::vector<int>& ptIDs2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold, int max_matches)
{
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> simple_matches;
  std::vector<int>::iterator it;
  int train_idx;
  matcher.match(descriptors1, descriptors2, simple_matches);
  int nmatch = simple_matches.size();
  std::sort(simple_matches.begin(), simple_matches.end());

  if((max_matches < 0)||(max_matches > nmatch))
    max_matches = nmatch;
  for (unsigned k = 0; k < max_matches; k++)
  {
    if ((simple_matches[k].distance > threshold))
      break;
    it = find(matching_indices_2.begin(),matching_indices_2.end(),simple_matches[k].trainIdx);
    if (it==matching_indices_2.end()&&ptIDs1[simple_matches[k].queryIdx]<0&&ptIDs2[simple_matches[k].trainIdx]<0)
    {
      matching_indices_1.push_back(simple_matches[k].queryIdx);
      matching_indices_2.push_back(simple_matches[k].trainIdx);
    }
  }
}


bool triangulate(cv::Point3d& pt_out, Keyframe* kf1, Keyframe* kf2, int idx1, int idx2)
{
  //http://www.iim.cs.tut.ac.jp/~kanatani/papers/sstriang.pdf
  //(iterative method for higher order aproximation)
  /* get coordinates */
  cv::Mat drone2world1, drone2world2, cam2world1, cam2world2, origin1, origin2, P0, P1;
  cv::Mat T1, T2, K1, K2, F;
  cv::Mat pt1_h, pt2_h, u, P, x_hat, x1_hat, temp, x_tilde, x1_tilde;
  cv::Mat V, epsil, mult;
  cv::Point2d pt1, pt2;
  boris_drone::Pose3D pose1, pose2;
  Camera cam1, cam2;
  std::vector<cv::Point2d> cam0pnts,cam1pnts;
  cv::Mat pnts3D(4,1,CV_64F);
  pt1 = kf1->img_points[idx1];  pose1 = kf1->pose;  cam1 = kf1->camera;
  pt2 = kf2->img_points[idx2];  pose2 = kf2->pose;  cam2 = kf2->camera;
  double E, E0, f, den ;
  //getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  //getCameraPositionMatrices(pose2, cam2world2, origin2, true);
  getCameraPositionMatrices(pose1, drone2world1, origin1, true);
  getCameraPositionMatrices(pose2, drone2world2, origin2, true);
  cam2world1 = drone2world1*cam1.get_R();
  cam2world2 = drone2world2*cam2.get_R();
  K1 = cam1.get_K();
  K2 = cam2.get_K();
  T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin1.at<double>(0,0),
                                  0, 1, 0, -origin1.at<double>(1,0),
                                  0, 0, 1, -origin1.at<double>(2,0)
  );
  T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin2.at<double>(0,0),
                                  0, 1, 0, -origin2.at<double>(1,0),
                                  0, 0, 1, -origin2.at<double>(2,0)
  );

  //These are the camera projection matrices:
  // If p is a world point in 3D in the world coordinates,
  // then P0*p are the (homogenous) image coordinates of the world point
  P0 = K1*cam2world1.t()*T1;
  P1 = K2*cam2world2.t()*T2;

  F = (cv::Mat_<double>(3, 3)); // Fundamental Matrix3x3
  getFundamentalMatrix(P0,P1,F);

  pt1_h = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);
  pt2_h = (cv::Mat_<double>(3,1) << pt2.x, pt2.y, 1);

  u = (cv::Mat_<double>(9,1) << F.at<double>(0,0),F.at<double>(0,1),F.at<double>(0,2),
                                F.at<double>(1,0),F.at<double>(1,1),F.at<double>(1,2),
                                F.at<double>(2,0),F.at<double>(2,1),F.at<double>(2,2));
  P = (cv::Mat_<double>(3,3) << 1,0,0, 0,1,0, 0,0,0);

  f = 250;
  f = 1;

  x_hat  = pt1_h.clone();
  x1_hat = pt2_h.clone();

  x_tilde  = (cv::Mat_<double>(3,1) << 0,0,0);
  x1_tilde = (cv::Mat_<double>(3,1) << 0,0,0);

  int k = 0; E = 10; E0 = 0;
  while ((abs(E - E0)> 0.001)&&(k<10))
  {
    if(k==9)
      ROS_INFO("reached max iterations in triangulation");
    k = k+1;
    E0 = E;
    x_hat  = pt1_h - x_tilde;
    x1_hat = pt2_h - x1_tilde;
    getVandEpsil(V, epsil, x_hat, x1_hat, x_tilde, x1_tilde, f);
    temp = u.t()*V*u;
    den  = temp.at<double>(0,0);
    temp = u.t()*epsil;
    mult =  P*temp.at<double>(0,0)/den;
    x_tilde  = mult*F    *x1_hat;
    x1_tilde = mult*F.t()*x_hat ;
    E =  x_tilde.at<double>(0,0) *x_tilde.at<double>(0,0)  + x_tilde.at<double>(1,0) *x_tilde.at<double>(1,0)
       + x1_tilde.at<double>(0,0)*x1_tilde.at<double>(0,0) + x1_tilde.at<double>(1,0)*x1_tilde.at<double>(1,0);
  }
  if ((x_hat.at<double>(2,0)!=1)||(x1_hat.at<double>(2,0)!=1))
    ROS_WARN("x_hat and x1_hat's 3rd coordinate shoudl be 1 but they are: %f and %f",
                                          x_hat.at<double>(2,0),x1_hat.at<double>(2,0));
  cv::Point2d p1,p2;
  p1.x = x_hat.at<double>(0,0);
  p1.y = x_hat.at<double>(1,0);
  p2.x = x1_hat.at<double>(0,0);
  p2.y = x1_hat.at<double>(1,0);

  cam0pnts.push_back(p1);
  cam1pnts.push_back(p2);

  cv::triangulatePoints(P0,P1,cam0pnts,cam1pnts,pnts3D);

  pt_out.x = pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0);
  pt_out.y = pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0);
  pt_out.z = pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0);

  return true;
}

bool triangulate_dlt(cv::Point3d& pt_out, Keyframe* kf1, Keyframe* kf2, int idx1, int idx2)
{
  cv::Mat drone2world1,drone2world2,cam2world1, cam2world2, origin1, origin2, P0, P1;
  cv::Mat T1, T2, K1, K2, F;
  cv::Point2d pt1, pt2;
  boris_drone::Pose3D pose1, pose2;
  Camera cam1, cam2;
  std::vector<cv::Point2d> cam0pnts,cam1pnts;
  cv::Mat pnts3D(4,1,CV_64F);
  pt1 = kf1->img_points[idx1];  pose1 = kf1->pose;  cam1 = kf1->camera;
  pt2 = kf2->img_points[idx2];  pose2 = kf2->pose;  cam2 = kf2->camera;
  //getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  //getCameraPositionMatrices(pose2, cam2world2, origin2, true);
  getCameraPositionMatrices(pose1, drone2world1, origin1, true);
  getCameraPositionMatrices(pose2, drone2world2, origin2, true);
  cam2world1 = drone2world1*cam1.get_R();
  cam2world2 = drone2world2*cam2.get_R();
  K1 = cam1.get_K();
  K2 = cam2.get_K();
  T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin1.at<double>(0,0),
                                  0, 1, 0, -origin1.at<double>(1,0),
                                  0, 0, 1, -origin1.at<double>(2,0)
  );
  T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin2.at<double>(0,0),
                                  0, 1, 0, -origin2.at<double>(1,0),
                                  0, 0, 1, -origin2.at<double>(2,0)
  );
  P0 = K1*cam2world1.t()*T1;
  P1 = K2*cam2world2.t()*T2;
  cam0pnts.push_back(pt1);
  cam1pnts.push_back(pt2);
  cv::triangulatePoints(P0,P1,cam0pnts,cam1pnts,pnts3D);

  pt_out.x = pnts3D.at<double>(0,0)/pnts3D.at<double>(3,0);
  pt_out.y = pnts3D.at<double>(1,0)/pnts3D.at<double>(3,0);
  pt_out.z = pnts3D.at<double>(2,0)/pnts3D.at<double>(3,0);
  return true;
}

bool triangulate_midpoint(cv::Point3d& pt_out, Keyframe *kf1, Keyframe *kf2, int idx1, int idx2)
{
  //Midpoint method : http://geomalgorithms.com/a07-_distance.html
  double angleThresh = PI/40;

  /* get coordinates */
  cv::Mat drone2world1,drone2world2,cam2world1, cam2world2, origin1, origin2;
  cv::Point2d pt1,pt2;
  Camera cam1, cam2;
  boris_drone::Pose3D pose1, pose2;
  pt1 = kf1->img_points[idx1];  pose1 = kf1->pose;  cam1 = kf1->camera;
  pt2 = kf2->img_points[idx2];  pose2 = kf2->pose;  cam2 = kf2->camera;
  //getCameraPositionMatrices(pose1, cam2world1, origin1, true);
  //getCameraPositionMatrices(pose2, cam2world2, origin2, true);
  getCameraPositionMatrices(pose1, drone2world1, origin1, true);
  getCameraPositionMatrices(pose2, drone2world2, origin2, true);
  cam2world1 = drone2world1*cam1.get_R();
  cam2world2 = drone2world2*cam2.get_R();

  /* compute point coordinates in calibrated image coordinates (=rays in camera coordinates) */
  cv::Mat ray1_cam = (cv::Mat_<double>(3, 1) << (pt1.x - cam1.cx) / cam1.fx,
                                                (pt1.y - cam1.cy) / cam1.fy, 1);
  cv::Mat ray2_cam = (cv::Mat_<double>(3, 1) << (pt2.x - cam2.cx) / cam2.fx,
                                                (pt2.y - cam2.cy) / cam2.fy, 1);
  /* convert to world coordinates */
  cv::Mat ray1 = cam2world1 * ray1_cam;
  cv::Mat ray2 = cam2world2 * ray2_cam;

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


inline void getVandEpsil(cv::Mat& V, cv::Mat& epsil, cv::Mat& x_hat, cv::Mat& x1_hat,
  cv::Mat& x_tilde, cv::Mat& x1_tilde, double f)
{
  double x,y,x1,y1,x2,y2,x12,y12,xt,x1t,yt,y1t;
  x   = x_hat.at<double>(0,0)   ;  y   = x_hat.at<double>(1,0)   ;
  x1  = x1_hat.at<double>(0,0)  ;  y1  = x1_hat.at<double>(1,0)  ;
  xt  = x_tilde.at<double>(0,0) ;  yt  = x_tilde.at<double>(1,0) ;
  x1t = x1_tilde.at<double>(0,0);  y1t = x1_tilde.at<double>(1,0);
  x2 = x*x; x12 = x1*x1;  y2 = y*y; y12 = y1*y1;

  V=(cv::Mat_<double>(9,9) <<
     x2+x12,x1*y1, f*x1,  x*y,   0,     0,   f*x,0,  0,
     x1*y1, x2+y12,f*y1,  0,     x*y,   0,   0,  f*x,0,
     f*x1,  f*y1,  f*f,   0,     0,     0,   0,  0,  0,
     x*y,   0,     0,     y2+x12,x1*y1, f*x1,f*y,0,  0,
     0,     x*y,   0,     x1*y1, y2+y12,f*y1,0,  f*y,0,
     0,     0,     0,     f*x1,  f*y1,  f*f, 0  ,0,  0,
     f*x,   0,     0,     f*y,   0,     0,   f*f,0,  0,
     0,     f*x,   0,     0,     f*y,   0,   0,  f*f,0,
     0,0,0,               0,0,0,             0,0,0);
  epsil = (cv::Mat_<double>(9,1) <<
           x*x1 + x1*xt + x*x1t,
           x*y1 + y1*xt + x*y1t,
           (x   + xt)*f        ,
           y*x1 + x1*yt + y*x1t,
           y*y1 + y1*yt + y*y1t,
           (y   + yt)*f        ,
           (x1  + x1t)*f       ,
           (y1  + y1t)*f       ,
           f*f                 );
}
