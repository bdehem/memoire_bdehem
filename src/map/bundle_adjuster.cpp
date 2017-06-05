
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.
#include <boris_drone/map/bundle_adjuster.h>

BALProblem::BALProblem(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  bundleMsgPtr_     = bundlePtr;
  num_cameras_      = bundlePtr->num_cameras;
  num_points_       = bundlePtr->num_points;
  num_observations_ = bundlePtr->num_observations;
  num_parameters_   = 6 * num_cameras_ + 3 * num_points_;
  point_index_  = new int[num_observations_];
  camera_index_ = new int[num_observations_];
  fixed_cams_   = new bool[num_cameras_];
  observations_ = new double[2 * num_observations_];
  parameters_   = new double[num_parameters_];

  //Mapping keyframe and point IDs to indices for the BA
  std::map<int,int> kfID_to_idx;
  std::map<int,int> ptID_to_idx;
  for(int i = 0; i < num_cameras_; ++i)
    kfID_to_idx[bundlePtr->keyframes_ID[i]] = i;
  for(int i = 0; i < num_points_ ; ++i)
    ptID_to_idx[bundlePtr->points_ID[i]] = i;

  for (int i = 0; i < num_observations_; ++i) {
    camera_index_[i]     = kfID_to_idx[bundlePtr->observations[i].kfID];
    point_index_[i]      = ptID_to_idx[bundlePtr->observations[i].ptID];
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
    fixed_cams_[i] = bundlePtr->fixed_cams[i];
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
  delete[] fixed_cams_;
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

struct SemiFixedCameraError {
  SemiFixedCameraError(double x, double y, double z, double rotX, double rotY, double rotZ)
      : x(x), y(y), z(z), rotX(rotX), rotY(rotY), rotZ(rotZ) {}
  template <typename T>
  bool operator()(const T* const camera,
                  T* residuals) const {
    double weight_x = 0.1  ; double weight_rotX = 15.0;
    double weight_y = 0.1  ; double weight_rotY = 15.0;
    double weight_z = 15.0 ; double weight_rotZ = 0.1 ;

    residuals[0] = weight_x    * (camera[3] - x);
    residuals[1] = weight_y    * (camera[4] - y);
    residuals[2] = weight_z    * (camera[5] - z);
    residuals[3] = weight_rotX * (camera[0] - rotX);
    residuals[4] = weight_rotY * (camera[1] - rotY);
    residuals[5] = weight_rotZ * (camera[2] - rotZ);
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double x, const double y, const double z,
                    const double rotX, const double rotY, const double rotZ) {
    return (new ceres::AutoDiffCostFunction<SemiFixedCameraError, 6, 6>(
                new SemiFixedCameraError(x, y, z, rotX, rotY, rotZ)));
  }
  double x; double y; double z; double rotX; double rotY; double rotZ;
};


struct SemiFixedCameraError2 {
  SemiFixedCameraError2(double z, double rotX, double rotY)
      : z(z), rotX(rotX), rotY(rotY) {}
  template <typename T>
  bool operator()(const T* const camera,
                  T* residuals) const {
    double weight_z    = 15.0 ;
    double weight_rotX = 15.0;
    double weight_rotY = 15.0;
    residuals[0] = weight_z    * (camera[5] - z);
    residuals[1] = weight_rotX * (camera[0] - rotX);
    residuals[2] = weight_rotY * (camera[1] - rotY);
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double z, const double rotX, const double rotY) {
    return (new ceres::AutoDiffCostFunction<SemiFixedCameraError2, 6, 6>(
                new SemiFixedCameraError2(z, rotX, rotY)));
  }
  double z; double rotX; double rotY;
};


BundleAdjuster::BundleAdjuster()
{
  bundle_channel = nh.resolveName("bundle");
  bundle_sub     = nh.subscribe(bundle_channel, 10, &BundleAdjuster::bundleCb, this);

  // Publishers
  bundled_channel = nh.resolveName("bundled");
  bundled_pub     = nh.advertise<boris_drone::BundleMsg>(bundled_channel, 1);

  ros::param::get("~bundle_adjustment_tol", tolerance);
}

void BundleAdjuster::publishBundle(const BALProblem& bal_problem, bool converged,
            std::vector<double>& cost_of_point, double time_taken)
{
  boris_drone::BundleMsg msg = *(bal_problem.bundleMsgPtr_);
  int ncam = bal_problem.num_cameras_ ;  int npt  = bal_problem.num_points_;
  msg.num_cameras = ncam              ;  msg.num_points  = npt;
  msg.cameras.resize(ncam)            ;  msg.points.resize(npt);
  msg.converged = converged;
  msg.time_taken = time_taken;
  msg.cost_of_point.resize(npt);
  for (int i = 0; i < ncam; ++i) {
    msg.cameras[i].rotX = bal_problem.parameters_[6*i + 0]*PI/180.0;
    msg.cameras[i].rotY = bal_problem.parameters_[6*i + 1]*PI/180.0;
    msg.cameras[i].rotZ = bal_problem.parameters_[6*i + 2]*PI/180.0;
    msg.cameras[i].x    = bal_problem.parameters_[6*i + 3];
    msg.cameras[i].y    = bal_problem.parameters_[6*i + 4];
    msg.cameras[i].z    = bal_problem.parameters_[6*i + 5];
  }
  for (int i = 0; i < npt; ++i) {
    msg.points[i].x = bal_problem.parameters_[6*ncam + 3*i + 0];
    msg.points[i].y = bal_problem.parameters_[6*ncam + 3*i + 1];
    msg.points[i].z = bal_problem.parameters_[6*ncam + 3*i + 2];
    msg.cost_of_point[i] = cost_of_point[i];
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
  bool is_first_pass = bundlePtr->is_first_pass;

  double* camera_print;
  ROS_INFO("Cameras before BA:");
  for (int i = 0; i< bal_problem.num_cameras_; i++)
  {
    camera_print = bal_problem.mutable_camera(i);
    ROS_INFO("R : roll = %.2f, pitch = %.2f, yaw = %.2f",
    camera_print[0]*PI/180.0,camera_print[1]*PI/180.0,camera_print[2]*PI/180.0);
    ROS_INFO("T : x    = %.2f, y     = %.2f, z   = %.2f",camera_print[3],camera_print[4],camera_print[5]);
  }


  /*
  double* camera_print  = bal_problem.mutable_camera(0);
  double* camera_print2 = bal_problem.mutable_camera(1);


  */

  double x, y;
  double* camera;
  double* point;
  ceres::Problem problem;
  ceres::CostFunction* cost_function;
  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    camera = bal_problem.mutable_camera_for_observation(i);
    point  = bal_problem.mutable_point_for_observation(i);
    x = bal_problem.observations()[2 * i + 0];
    y = bal_problem.observations()[2 * i + 1];
    //TODO this is temporary
    is_first_pass = true;
    cost_function = SnavelyReprojectionError::Create(x, y);
    ceres::LossFunction* loss_function = is_first_pass ? new ceres::HuberLoss(1.0) : NULL;
    problem.AddResidualBlock(cost_function, loss_function, camera, point);
  }
  problem.SetParameterBlockConstant(bal_problem.mutable_camera(0));

  //This causes drift: TODO fix
  for (int i = 1; i < bal_problem.num_cameras_; ++i)
  {
    camera = bal_problem.mutable_camera_for_observation(i);
    cost_function = SemiFixedCameraError::Create(camera[3],camera[4],camera[5],camera[0],camera[1],camera[2]);
    //cost_function = SemiFixedCameraError2::Create(camera[5],camera[0],camera[1]);
    problem.AddResidualBlock(cost_function, NULL, camera);
  }
  std::vector<double> cost_of_point;
  printResiduals(problem, bal_problem, cost_of_point);

  ceres::Solver::Options options;
  options.max_num_iterations = is_first_pass? 30 : 50;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  //TODO this is temporary
  options.minimizer_progress_to_stdout = true;
  options.function_tolerance = tolerance;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  bool converged = (summary.termination_type == ceres::CONVERGENCE);
  double time_taken = summary.total_time_in_seconds;

  ROS_INFO("Cameras after BA:");
  for (int i = 0; i< bal_problem.num_cameras_; i++)
  {
    camera_print = bal_problem.mutable_camera(i);
    ROS_INFO("R : roll = %.2f, pitch = %.2f, yaw = %.2f",
    camera_print[0]*PI/180.0,camera_print[1]*PI/180.0,camera_print[2]*PI/180.0);
    ROS_INFO("T : x    = %.2f, y     = %.2f, z   = %.2f",camera_print[3],camera_print[4],camera_print[5]);
  }
  printResiduals(problem, bal_problem, cost_of_point);

  publishBundle(bal_problem, converged, cost_of_point, time_taken);


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

void BundleAdjuster::printResiduals(ceres::Problem& problem, BALProblem& bal_problem,
      std::vector<double>& cost_of_point)
{
  double cost;
  std::vector<double> residuals;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, NULL, NULL);
  int nobs = bal_problem.num_observations_;
  int npt  = bal_problem.num_points_;
  std::vector<double> sqloss;
  std::vector<double> sqloss_of_point;
  std::vector<int> observations_of_point;
  sqloss.resize(nobs);
  sqloss_of_point.resize(npt,0.0);
  observations_of_point.resize(npt,0);
  cost_of_point.resize(npt);
  for (int i = 0; i<nobs; ++i)
  {
    sqloss[i] = residuals[2*i]*residuals[2*i] + residuals[2*i+1]*residuals[2*i+1];
    sqloss_of_point[bal_problem.point_index_[i]] += sqloss[i];
    observations_of_point[bal_problem.point_index_[i]]++;
  }
  for (int i = 0; i < npt; ++i)
  {
    cost_of_point[i] = sqloss_of_point[i]/(double)observations_of_point[i];
    //ROS_INFO("Pt %d: cost = %f nobs = %d cost/obs = %f",
    //i,sqloss_of_point[i],observations_of_point[i],cost_of_point[i]);
  }
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
