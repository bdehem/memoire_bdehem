struct ConstCameraReprojectionError {
 public:
  ConstCameraReprojectionError(
      const Eigen::Matrix<double, 3, 4>& projection_matrix,
      const Eigen::Vector2d& feature)
      : projection_matrix_(projection_matrix), feature_(feature) {}

  template <typename T>
  bool operator()(const T* input_point, T* reprojection_error) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > point(input_point);
    Eigen::Matrix<T,4,1> point_h;
    point_h(0) = point(0);
    point_h(1) = point(1);
    point_h(2) = point(2);
    point_h(3) = T(1.0);
    //point_h = point.homogeneous();

    // Multiply the point with the projection matrix, then perform homogeneous
    // normalization to obtain the 2D pixel location of the reprojection.
    const Eigen::Matrix<T, 2, 1> reprojected_pixel =
        (projection_matrix_.cast<T>() * point_h ).hnormalized();

    // Reprojection error is the distance from the reprojection to the observed
    // feature location.
    reprojection_error[0] = feature_[0] - reprojected_pixel[0];
    reprojection_error[1] = feature_[1] - reprojected_pixel[1];
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double* camera)
  {
    cv::Mat K = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                           0,     529.1, 182.2,
                                           0,     0,     1
    );
    cv::Mat R = rollPitchYawToRotationMatrix(camera[0]*PI/180.0,
                                             camera[1]*PI/180.0,
                                             camera[2]*PI/180.0
    );
    cv::Mat T = (cv::Mat_<double>(3, 4) << 1, 0, 0, -camera[3],
                                           0, 1, 0, -camera[4],
                                           0, 0, 1, -camera[5]
    );
    cv::Mat P = K*R.t()*T;
    Eigen::Matrix<double,3,4> projection_matrix;
    cv2eigen(P,projection_matrix);

    Eigen::Vector2d feature;
    feature(0) = observed_x;
    feature(1) = observed_y;
    return (new ceres::AutoDiffCostFunction<ConstCameraReprojectionError, 2, 3>(
                new ConstCameraReprojectionError(projection_matrix, feature)));
  }

 private:
  const Eigen::Matrix<double, 3, 4>& projection_matrix_;
  const Eigen::Vector2d& feature_;
};




/* Display stuff at start of bundle Cb

double realPoint[3] = {bundlePtr->points[0].x,bundlePtr->points[0].y,bundlePtr->points[0].z};

double* cam1 = bal_problem.parameters_;
double* cam2 = bal_problem.parameters_ + 6;
double R1[9];
double R2[9];
double camera_axis_angles1[3]; //angle-axis rotation
double camera_axis_angles2[3]; //angle-axis rotation
double point[3] = {2.0,0.0,0.0};
double p_cam[3];
double p[3];
cout << "cam1 [0-2]:" << endl;
cout << cam1[0] << "\t" << cam1[1] << "\t" << cam1[2] << endl;
cout << "cam1 [3-5]:" << endl;
cout << cam1[3] << "\t" << cam1[4] << "\t" << cam1[5] << endl;

//This function takes Euler angles in degrees
//It gives the same result as rollPitchYawToRotationMatrix in opencv_utils.cpp
ceres::EulerAnglesToRotationMatrix(cam1, 3, R1);
ceres::EulerAnglesToRotationMatrix(cam2, 3, R2);
cout << "R1:" << endl;
cout << R1[0] << "\t" << R1[1] << "\t" << R1[2] << endl;
cout << R1[3] << "\t" << R1[4] << "\t" << R1[5] << endl;
cout << R1[6] << "\t" << R1[7] << "\t" << R1[8] << endl;
ceres::RotationMatrixToAngleAxis(R1, camera_axis_angles1);
ceres::RotationMatrixToAngleAxis(R2, camera_axis_angles2);

//Translate point to get location from camera origin
p_cam[0] = realPoint[0] - cam1[3];
p_cam[1] = realPoint[1] - cam1[4];
p_cam[2] = realPoint[2] - cam1[5];

//Get rotated point
ceres::AngleAxisRotatePoint(camera_axis_angles1, p_cam, p);
cout << "p_in:" << endl;
cout << realPoint[0] << "\t" << realPoint[1] << "\t" << realPoint[2] << endl;
cout << "p_out:" << endl;
cout << p[0] << "\t" << p[1] << "\t" << p[2] << endl;
cout << "camera_axis_angles1:" << endl;
cout << camera_axis_angles1[0] << "\t" << camera_axis_angles1[1] << "\t" << camera_axis_angles1[2] << endl;

double predicted_x = 350.6 + (529.1*p[0] / p[2]);
double predicted_y = 182.2 + (529.1*p[1] / p[2]);

cout << "predicted_out1:" << endl;
cout << predicted_x << "\t" << predicted_y << endl;

//Translate point to get location from camera origin
p_cam[0] = realPoint[0] - cam2[3];
p_cam[1] = realPoint[1] - cam2[4];
p_cam[2] = realPoint[2] - cam2[5];
//Get rotated point
ceres::AngleAxisRotatePoint(camera_axis_angles2, p_cam, p);
predicted_x = 350.6 + (529.1*p[0] / p[2]);
predicted_y = 182.2 + (529.1*p[1] / p[2]);

cout << "predicted_out2:" << endl;
cout << predicted_x << "\t" << predicted_y << endl;

cout << "observed_in:" << endl;
cout << bundlePtr->observations[0].x << "\t" << bundlePtr->observations[0].y << endl;

cout << "observed_in2:" << endl;
cout << bundlePtr->observations[1].x << "\t" << bundlePtr->observations[1].y << endl;
endDisplay stuff at start of bundle Cb */




//Old code when using 2 different types of costfunction
//if (bal_problem.camera_index_[i] == 0)
////if (false)
//{
//  ceres::CostFunction* cost_function = ConstCameraReprojectionError::Create(x, y, camera);
//  problem.AddResidualBlock(cost_function, NULL, point);//NULL: squared loss
//}
//else
//{
//  ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(x, y);
//  problem.AddResidualBlock(cost_function, NULL, camera, point);
//}
