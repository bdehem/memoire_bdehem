/*!
 *  This file is part of ucl_drone 2017.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Boris Dehem
 *  \date 2017
 */

#include <ucl_drone/map/camera.h>

Camera::Camera() {}

Camera::Camera(bool is_front)
{
  if (!is_front)
  {
    ROS_ERROR("Only front camera of the AR.Drone is currently implemented");
    return;
  }
  fx = 529.1; fy = 529.1;
  cx = 350.6; cy = 182.2;
  W  = 720  ; H = 360;
  roll  = -PI/2;
  pitch = 0;
  yaw   = -PI/2;

  K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0,  0, 1);
  R = (cv::Mat_<double>(3, 3) << 0,  0, 1, -1,  0, 0,  0, -1, 0);
  initPlanes();
}

cv::Mat Camera::get_K() {return K;}

//cv::Mat Camera::get_R() {return R;}
cv::Mat Camera::get_R() { return rollPitchYawToRotationMatrix(roll, pitch, yaw);}

void Camera::initPlanes()
{
  float thres = 0.95;
  cv::Mat top_left     = (cv::Mat_<double>(3, 1) <<   -cx /fx,  -cy /fy, thres);
  cv::Mat top_right    = (cv::Mat_<double>(3, 1) << (W-cx)/fx,  -cy /fy, thres);
  cv::Mat bottom_right = (cv::Mat_<double>(3, 1) << (W-cx)/fx,(H-cy)/fy, thres);
  cv::Mat bottom_left  = (cv::Mat_<double>(3, 1) <<   -cx /fx,(H-cy)/fy, thres);
  cam_plane_top    = top_left.cross(top_right);
  cam_plane_right  = top_right.cross(bottom_right);
  cam_plane_bottom = bottom_right.cross(bottom_left);
  cam_plane_left   = bottom_left.cross(top_left);
  ROS_DEBUG("camera planes initilized");
}
