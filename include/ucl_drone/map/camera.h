/*!
 *  \file camera.h
 *  \brief File defining a camera object
 *  \author Boris Dehem
 *  \date 2017
 */

#ifndef ucl_drone_CAMERA_H
#define ucl_drone_CAMERA_H

#include <ucl_drone/ucl_drone.h>
#include <ucl_drone/opencv_utils.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>


struct Camera
{
private:
 /**
  * This method initializes planes defining the visible area from the camera (according to camera parameters)
  */
  void initPlanes();
  cv::Mat K; //!< Intrinsic camera matrix
  cv::Mat R; //!< Rotation matrix camera to drone

public:
  double fx;    //!< Horizontal focal length
  double fy;    //!< Vertical focal length
  double cx;    //!< Image center x-coordinate
  double cy;    //!< Image center y-coordinate
  double W;     //!< Image width
  double H;     //!< Image height
  double roll;  //!< Roll angle of camera to drone rotation
  double pitch; //!< Pitch angle of camera to drone rotation
  double yaw;   //!< Yaw angle of camera to drone rotation

  cv::Mat cam_plane_top;    //!< Plane defining top edge of field of vision in camera coordinates
  cv::Mat cam_plane_bottom; //!< Plane defining bottom edge of field of vision in camera coordinates
  cv::Mat cam_plane_left;   //!< Plane defining left edge of field of vision in camera coordinates
  cv::Mat cam_plane_right;  //!< Plane defining right edge of field of vision in camera coordinates

  Camera(); //!< Empty Constructor

 /**
  * Constructor
  * @param[in] is_front If true, camera parameters of AR.Drone's front camera are taken
  */
  Camera(bool is_front);

  cv::Mat get_K(); //!< Get camera intrinsic matrix
  cv::Mat get_R(); //!< Get camera rotation matrix (to drone)

};

#endif /*ucl_drone_CAMERA_H*/
