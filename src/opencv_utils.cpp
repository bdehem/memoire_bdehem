/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  http://answers.opencv.org/question/64427/position-and-rotation-of-two-cameras-which-functions-i-need-in-what-order/
 *  https://gist.github.com/royshil/7087bc2560c581d443bc
 */

#include <boris_drone/opencv_utils.h>

//! Rotation matrix about the X axis
cv::Mat rotationMatrixX(const double angle)
{
  cv::Mat Rx = (cv::Mat_< double >(3, 3) << 1.0, 0.0,         0.0,
                                            0.0, cos(angle), -sin(angle),
                                            0.0, sin(angle),  cos(angle));
  return Rx;
}

//! Rotation matrix about the Y axis
cv::Mat rotationMatrixY(const double angle)
{
  cv::Mat Ry = (cv::Mat_< double >(3, 3) << cos(angle), 0.0, sin(angle),
                                            0.0,        1.0, 0.0,
                                           -sin(angle), 0.0, cos(angle));
  return Ry;
}

//! Rotation matrix about the Z axis
cv::Mat rotationMatrixZ(const double angle)
{
  cv::Mat Rz = (cv::Mat_< double >(3, 3) << cos(angle), -sin(angle), 0.0,
                                            sin(angle),  cos(angle), 0.0,
                                            0.0,         0.0,        1.0);
  return Rz;
}

//! Rotation matrix given RPY angles
//If yaw angle is turned around z axis, then pitch angle around y axis, and finally roll angle around x axis,
//the result is a rotation matrix whose columns are the coordinates of the new base expressed in the old base.
//Example:
// pt_world = 3D vector in world coordinates
// A = rollPitchYawToRotationMatrix(roll,pitch,yaw)
// pt_cam = A.t()*pt_world;
// Where roll, pitch and yaw are the angles to rotate to obtain a coordinate system where
// z points forward, x points right, y points down from the point of view of the camera
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw)
{
  cv::Mat Rx = rotationMatrixX(roll);
  cv::Mat Ry = rotationMatrixY(pitch);
  cv::Mat Rz = rotationMatrixZ(yaw);

  return Rz * Ry * Rx;
}

//This function gives the same reslt as the transpose of the above function,
//but it is clearer to have two different functions
cv::Mat rollPitchYawToChangeBaseMatrix(const double roll, const double pitch, const double yaw)
{
  cv::Mat Rx = rotationMatrixX(-roll);  //-angles
  cv::Mat Ry = rotationMatrixY(-pitch);
  cv::Mat Rz = rotationMatrixZ(-yaw);

  return Rx * Ry * Rz;//order inversed
}

void getCameraPositionMatrices(const boris_drone::Pose3D& pose, cv::Mat& R, cv::Mat& T, bool front)
{
  R = rollPitchYawToRotationMatrix(pose.rotX, pose.rotY, pose.rotZ);
  T = (cv::Mat_< double >(3, 1) << pose.x, pose.y, pose.z);
}

//! Transofmation (rotation and translation) matrix given RPY angles and translation lengths
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz)
{
  cv::Mat Rt = (cv::Mat_<double>(3, 4) << rot.at<double>(0, 0), rot.at<double>(0, 1),
                rot.at<double>(0, 2), tx, rot.at<double>(1, 0), rot.at<double>(1, 1),
                rot.at<double>(1, 2), ty, rot.at<double>(2, 0), rot.at<double>(2, 1),
                rot.at<double>(2, 2), tz);
  return Rt;
}

void debugRTMatrix(cv::Mat Rt)
{
  ROS_DEBUG("Rt = \n[ %f \t %f \t %f \t %f \n  %f \t %f \t %f \t %f \n  %f \t %f \t %f \t %f ]",
            Rt.at< double >(0, 0), Rt.at< double >(0, 1), Rt.at< double >(0, 2),
            Rt.at< double >(0, 3), Rt.at< double >(1, 0), Rt.at< double >(1, 1),
            Rt.at< double >(1, 2), Rt.at< double >(1, 3), Rt.at< double >(2, 0),
            Rt.at< double >(2, 1), Rt.at< double >(2, 2), Rt.at< double >(2, 3));
}

//! Convert vector of opencv keypoints in a vector of opencv 2D points
std::vector< cv::Point2f > Points(const std::vector< cv::KeyPoint >& keypoints)
{
  std::vector< cv::Point2f > result;
  for (unsigned i = 0; i < keypoints.size(); i++)
  {
    result.push_back(cv::Point2f(keypoints[i].pt.x, keypoints[i].pt.y));
  }
  return result;
}


double getSqDist(boris_drone::Pose3D pose1, boris_drone::Pose3D pose2)
{
  return ((pose1.x-pose2.x) * (pose1.x-pose2.x))
       + ((pose1.y-pose2.y) * (pose1.y-pose2.y))
       + ((pose1.z-pose2.z) * (pose1.z-pose2.z));
}

double angleBetween(const cv::Mat &p1, const cv::Mat &p2)
{
    double len1 = sqrt(  p1.at<double>(0,0)*p1.at<double>(0,0)
                       + p1.at<double>(1,0)*p1.at<double>(1,0)
                       + p1.at<double>(2,0)*p1.at<double>(2,0));

    double len2 = sqrt(  p2.at<double>(0,0)*p2.at<double>(0,0)
                       + p2.at<double>(1,0)*p2.at<double>(1,0)
                       + p2.at<double>(2,0)*p2.at<double>(2,0));

    double dot =  p1.at<double>(0,0)* p2.at<double>(0,0)
                + p1.at<double>(1,0)* p2.at<double>(1,0)
                + p1.at<double>(2,0)* p2.at<double>(2,0);

    double a = dot / (len1 * len2);

    if (a >= 1.0)
        return 0.0;
    else if (a <= -1.0)
        return PI;
    else
        return acos(a);
}


void getFundamentalMatrix(const cv::Mat &P1, const cv::Mat &P2, cv::Mat F)
{
  cv::Mat_<double> X[3];
  vconcat( P1.row(1), P1.row(2), X[0] );
  vconcat( P1.row(2), P1.row(0), X[1] );
  vconcat( P1.row(0), P1.row(1), X[2] );

  cv::Mat_<double> Y[3];
  vconcat( P2.row(1), P2.row(2), Y[0] );
  vconcat( P2.row(2), P2.row(0), Y[1] );
  vconcat( P2.row(0), P2.row(1), Y[2] );

  cv::Mat_<double> XY;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      vconcat(X[j], Y[i], XY);
      F.at<double>(i, j) = determinant(XY);
    }
}
