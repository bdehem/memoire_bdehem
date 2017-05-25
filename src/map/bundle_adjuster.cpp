
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.
#include <boris_drone/map/bundle_adjuster.h>

BALProblem::BALProblem(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  fixed_poses_      = bundlePtr->fixed_poses;
  num_cameras_      = bundlePtr->num_cameras;
  num_points_       = bundlePtr->num_points;
  num_observations_ = bundlePtr->num_observations;
  num_parameters_   = 6 * num_cameras_ + 3 * num_points_;
  point_index_  = new int[num_observations_];
  camera_index_ = new int[num_observations_];
  observations_ = new double[2 * num_observations_];
  parameters_   = new double[num_parameters_];

  for (int i = 0; i < num_observations_; ++i) {
    camera_index_[i] = bundlePtr->observations[i].camera_index;
    point_index_[i]  = bundlePtr->observations[i].point_index;
    observations_[2*i]   = bundlePtr->observations[i].x;
    observations_[2*i+1] = bundlePtr->observations[i].y;
  }
  for (int i = 0; i < num_cameras_; ++i) {
    parameters_[6*i + 0] = bundlePtr->cameras[i].rotX*180/PI;
    parameters_[6*i + 1] = bundlePtr->cameras[i].rotY*180/PI;
    parameters_[6*i + 2] = bundlePtr->cameras[i].rotZ*180/PI;
    parameters_[6*i + 3] = bundlePtr->cameras[i].x;
    parameters_[6*i + 4] = bundlePtr->cameras[i].y;
    parameters_[6*i + 5] = bundlePtr->cameras[i].z;
  }
  for (int i = 0; i < num_points_; ++i) {
    parameters_[6*num_cameras_ + 3*i + 0] = bundlePtr->points[i].x;
    parameters_[6*num_cameras_ + 3*i + 1] = bundlePtr->points[i].y;
    parameters_[6*num_cameras_ + 3*i + 2] = bundlePtr->points[i].z;
  }
}
BALProblem::~BALProblem() {
  delete[] point_index_;
  delete[] camera_index_;
  delete[] observations_;
  delete[] parameters_;
}
int BALProblem::num_observations()       const { return num_observations_;               }
const double* BALProblem::observations() const { return observations_;                   }
double* BALProblem::mutable_cameras()          { return parameters_;                     }
double* BALProblem::mutable_points()           { return parameters_  + 6 * num_cameras_; }
double* BALProblem::mutable_camera(int i)      { return mutable_cameras()  + 6 * i;      }
double* BALProblem::mutable_point(int i)       { return mutable_points()   + 3 * i;      }
double* BALProblem::mutable_camera_for_observation(int i) { return mutable_camera(camera_index_[i]);}
double* BALProblem::mutable_point_for_observation(int i)  { return mutable_point(point_index_[i]);  }

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 6 parameters: 3 for rotation, 3 for translation,
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}
  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    T cam2world[9];              //rotation matrix
    T camera_axis_angles[3];     //angle-axis rotation
    T p_cam[3];
    T p[3];
    //Convert
    ceres::EulerAnglesToRotationMatrix(camera, 3, cam2world);
    ceres::RotationMatrixToAngleAxis(cam2world, camera_axis_angles);
    //Translate point to get location from camera origin
    p_cam[0] = point[0] - camera[3];
    p_cam[1] = point[1] - camera[4];
    p_cam[2] = point[2] - camera[5];
    //Get rotated point. -1 because this is the rotation of cam2world (we need world2cam)
    ceres::AngleAxisRotatePoint(camera_axis_angles, p_cam, p);
    // TODO include fx,fy,cx,cy instead of hardcode
    //Remider:
    //K2 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
    //                                0,     529.1, 182.2,
    //                                0,     0,     1
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
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(
                new SnavelyReprojectionError(observed_x, observed_y)));
  }
  double observed_x;
  double observed_y;
};


BundleAdjuster::BundleAdjuster()
{
  bundle_channel = nh.resolveName("bundle");
  bundle_sub     = nh.subscribe(bundle_channel, 10, &BundleAdjuster::bundleCb, this);

  // Publishers
  bundled_channel = nh.resolveName("bundled");
  bundled_pub     = nh.advertise<boris_drone::BundleMsg>(bundled_channel, 1);
}

void BundleAdjuster::publishBundle(const BALProblem& bal_problem, bool converged)
{
  boris_drone::BundleMsg msg;
  msg.converged = converged;
  int ncam = bal_problem.num_cameras_ ;  int npt  = bal_problem.num_points_;
  msg.num_cameras = ncam              ;  msg.num_points  = npt;
  msg.cameras.resize(ncam)            ;  msg.points.resize(npt);

  for (int i = 0; i < ncam; ++i) {
    msg.cameras[i].rotX = bal_problem.parameters_[6*i + 0] ;
    msg.cameras[i].rotY = bal_problem.parameters_[6*i + 1] ;
    msg.cameras[i].rotZ = bal_problem.parameters_[6*i + 2] ;
    msg.cameras[i].x    = bal_problem.parameters_[6*i + 3] ;
    msg.cameras[i].y    = bal_problem.parameters_[6*i + 4] ;
    msg.cameras[i].z    = bal_problem.parameters_[6*i + 5] ;
  }
  for (int i = 0; i < npt; ++i) {
    msg.points[i].x = bal_problem.parameters_[6*ncam + 3*i + 0];
    msg.points[i].y = bal_problem.parameters_[6*ncam + 3*i + 1];
    msg.points[i].z = bal_problem.parameters_[6*ncam + 3*i + 2];
  }
  bundled_pub.publish(msg);
}

BundleAdjuster::~BundleAdjuster()
{
}


void BundleAdjuster::bundleCb(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  //Inspired by example code for bundle adjustment of Ceres (main function)
  //google::InitGoogleLogging(argv[0]);
  BALProblem bal_problem = BALProblem(bundlePtr);
  /*
  blabla
  double* camera_print  = bal_problem.mutable_camera(0);
  double* camera_print2 = bal_problem.mutable_camera(1);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                         0,     529.1, 182.2,
                                         0,     0,     1
  );

  cv::Mat R1 = rollPitchYawToRotationMatrix(camera_print[0]*PI/180.0,
                                            camera_print[1]*PI/180.0,
                                            camera_print[2]*PI/180.0
  );
  cv::Mat T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -camera_print[3],
                                          0, 1, 0, -camera_print[4],
                                          0, 0, 1, -camera_print[5]
  );
  cv::Mat R2 = rollPitchYawToRotationMatrix(camera_print2[0]*PI/180.0,
                                            camera_print2[1]*PI/180.0,
                                            camera_print2[2]*PI/180.0
  );
  cv::Mat T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -camera_print2[3],
                                          0, 1, 0, -camera_print2[4],
                                          0, 0, 1, -camera_print2[5]
  );
  cv::Mat P1 = K*R1.t()*T1;
  cv::Mat P2 = K*R2.t()*T2;
  Eigen::Matrix<double,3,4> projection_matrix1;
  Eigen::Matrix<double,3,4> projection_matrix2;
  cv2eigen(P1,projection_matrix1);
  cv2eigen(P2,projection_matrix2);
  std::cout << "camera_print" << std::endl;
  std::cout << camera_print[0]<<camera_print[1]<<camera_print[2] << std::endl;
  std::cout << "K" << std::endl;
  std::cout << K << std::endl;
  std::cout << "R" << std::endl;
  std::cout << R1 << std::endl;
  std::cout << "T" << std::endl;
  std::cout << T1 << std::endl;
  std::cout << "P" << std::endl;
  std::cout << P1 << std::endl;
  std::cout << "projection_matrix" << std::endl;
  std::cout << projection_matrix1 << std::endl;

  Eigen::Vector4d point3D_h;
  int num_cameras_ = bal_problem.num_cameras_;
  point3D_h(0) = bal_problem.parameters_[6*num_cameras_ + 0];
  point3D_h(1) = bal_problem.parameters_[6*num_cameras_ + 1];
  point3D_h(2) = bal_problem.parameters_[6*num_cameras_ + 2];
  point3D_h(3) = 1.0;
  Eigen::Vector2d feature2d1;
  Eigen::Vector2d feature2d2;
  feature2d1(0) = bal_problem.observations_[0];
  feature2d1(1) = bal_problem.observations_[1];
  feature2d2(0) = bal_problem.observations_[2];
  feature2d2(1) = bal_problem.observations_[3];

  Eigen::Vector3d pt_out_h1 = projection_matrix1*point3D_h;
  Eigen::Vector2d pt_out1   = pt_out_h1.hnormalized();
  Eigen::Vector3d pt_out_h2 = projection_matrix2*point3D_h;
  Eigen::Vector2d pt_out2   = pt_out_h2.hnormalized();
  std::cout << "point3D_h" << std::endl;
  std::cout << point3D_h << std::endl;
  std::cout << "pt_out1" << std::endl;
  std::cout << pt_out1 << std::endl;
  std::cout << "feature2d1" << std::endl;
  std::cout << feature2d1 << std::endl;
  std::cout << "pt_out2" << std::endl;
  std::cout << pt_out2 << std::endl;
  std::cout << "feature2d2" << std::endl;
  std::cout << feature2d2 << std::endl;





  std::cout << "What does residual block see?" << std::endl;
  int i = 0;
  double* input_point = bal_problem.mutable_point_for_observation(i);
  double reprojection_error[2];
  Eigen::Map<const Eigen::Matrix<double, 3, 1> > mapped_point(input_point);
  Eigen::Matrix<double,4,1> point_h;
  point_h(0) = mapped_point(0);
  point_h(1) = mapped_point(1);
  point_h(2) = mapped_point(2);
  point_h(3) = 1.0;
  const Eigen::Matrix<double, 2, 1> reprojected_pixel =
  (projection_matrix1 * point_h ).hnormalized();

  // Reprojection error is the distance from the reprojection to the observed
  // feature location.
  reprojection_error[0] = feature2d1[0] - reprojected_pixel[0];
  reprojection_error[1] = feature2d1[1] - reprojected_pixel[1];
  std::cout << "feature2d1" << std::endl;
  std::cout << feature2d1 << std::endl;
  std::cout << "reprojected_pixel" << std::endl;
  std::cout << reprojected_pixel << std::endl;
  std::cout << "reprojection_error" << std::endl;
  std::cout << reprojection_error[0] << std::endl;
  std::cout << reprojection_error[1] << std::endl;
  */

  double x, y;
  double* camera;
  double* point;
  ceres::Problem problem;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    camera = bal_problem.mutable_camera_for_observation(i);
    point  = bal_problem.mutable_point_for_observation(i);
    x = bal_problem.observations()[2 * i + 0];
    y = bal_problem.observations()[2 * i + 1];

    ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(x, y);
    problem.AddResidualBlock(cost_function, NULL, camera, point);
  }
  problem.SetParameterBlockConstant(bal_problem.mutable_camera(0));
  if (bal_problem.fixed_poses_)
    for (int i = 1; i < bal_problem.num_cameras_; ++i)
      problem.SetParameterBlockConstant(bal_problem.mutable_camera(i));

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  ceres::TerminationType tType = summary.termination_type;
  bool converged = (tType == ceres::CONVERGENCE);
  ROS_INFO("tType converged is :%d",converged);
  publishBundle(bal_problem, converged);
  /*
  More printing

  point3D_h(0) = bal_problem.parameters_[6*num_cameras_ + 0];
  point3D_h(1) = bal_problem.parameters_[6*num_cameras_ + 1];
  point3D_h(2) = bal_problem.parameters_[6*num_cameras_ + 2];
  point3D_h(3) = 1.0;

  pt_out_h1 = projection_matrix1*point3D_h;
  pt_out1   = pt_out_h1.hnormalized();
  std::cout << "point3D_h" << std::endl;
  std::cout << point3D_h << std::endl;
  std::cout << "pt_out1" << std::endl;
  std::cout << pt_out1 << std::endl;
  std::cout << "feature2d1" << std::endl;
  std::cout << feature2d1 << std::endl;

  pt_out_h2 = projection_matrix2*point3D_h;
  pt_out2   = pt_out_h2.hnormalized();
  std::cout << "point3D_h" << std::endl;
  std::cout << point3D_h << std::endl;
  std::cout << "pt_out2" << std::endl;
  std::cout << pt_out2 << std::endl;
  std::cout << "feature2d2" << std::endl;
  std::cout << feature2d2 << std::endl;




  std::cout << "What does residual block see now?" << std::endl;
  int j = 0;
  double* input_point_after = bal_problem.mutable_point_for_observation(j);
  double reprojection_error_after[2];
  Eigen::Map<const Eigen::Matrix<double, 3, 1> > mapped_point_after(input_point_after);
  Eigen::Matrix<double,4,1> point_h_after;
  point_h_after(0) = mapped_point_after(0);
  point_h_after(1) = mapped_point_after(1);
  point_h_after(2) = mapped_point_after(2);
  point_h_after(3) = 1.0;
  const Eigen::Matrix<double, 2, 1> reprojected_pixel_after =
  (projection_matrix1 * point_h_after ).hnormalized();

  // Reprojection error is the distance from the reprojection to the observed
  // feature location.
  reprojection_error_after[0] = feature2d1[0] - reprojected_pixel_after[0];
  reprojection_error_after[1] = feature2d1[1] - reprojected_pixel_after[1];
  std::cout << "feature2d1" << std::endl;
  std::cout << feature2d1 << std::endl;
  std::cout << "reprojected_pixel_after" << std::endl;
  std::cout << reprojected_pixel_after << std::endl;
  std::cout << "reprojection_error_after" << std::endl;
  std::cout << reprojection_error_after[0] << std::endl;
  std::cout << reprojection_error_after[1] << std::endl;

  */
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "bundle_adjuster");

  BundleAdjuster bundler_node;

  ros::Rate r(3);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
