
#ifndef boris_drone_CAMERA_H
#define boris_drone_CAMERA_H

#include <boris_drone/boris_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>


struct Camera
{
private:
  double fx, fy, cx, cy; //focal lengths and image centers
  double W, H; //width ad height
  //! This method initializes planes defining the visible area from the camera (according to camera parameters)
  void initPlanes();
  cv::Mat K; //Intrinsic camera matrix
  cv::Mat R; //Rotation matrix camera to drone

public:

  cv::Mat cam_plane_top;
  cv::Mat cam_plane_bottom;
  cv::Mat cam_plane_left;
  cv::Mat cam_plane_right;

  Camera();
  Camera(bool is_front);
  cv::Mat get_K();
  cv::Mat get_R();

};

#endif /*boris_drone_CAMERA_H*/
