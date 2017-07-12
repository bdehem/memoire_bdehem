/*!
 *  \file target.h
 *  \brief Header file for the Target class which wraps all procedures to detect a predfined target
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone.
 */

#ifndef boris_drone_TARGET_H
#define boris_drone_TARGET_H

#include <boris_drone/profiling.h>

#include <boost/shared_ptr.hpp>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

#include <boris_drone/boris_drone.h>

// vision
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/ocl.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <opencv2/video/tracking.hpp>

//messages
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/ProcessedImageMsg.h>
#include <boris_drone/KeyPoint.h>

#include <boris_drone/constants/feature_types.h>


//! Filename to the target from within the package
static const std::string TARGET_RELPATH = "/target/target_bottom.png";

#ifdef DEBUG_TARGET
static const std::string OPENCV_WINDOW = "Object matches";
#endif

// #define DEBUG_TARGET // if defined a window with target matches is displayed

// #define DEBUG_PROJECTION  // if defined print relative errors of projection for the target

/*!
 *  \class Target
 *  \brief Provide tools to track the presence of a target
 */
class Target
{
private:
  cv::Mat image;                         //! Picture of the target as read at $TARGET_RELPATH$
  std::vector<cv::KeyPoint> keypoints;   //! keypoints detected on the target picture
  cv::Mat descriptors;                   //! target keypoints descriptors
  std::vector<cv::Point2f> centerAndCorners;  //! position of the center and the corners of the
                                              //! target
  cv::FlannBasedMatcher matcher;  //! wrapper object to the FLANN library to perform matching with
                                  //! the video pictures

public:
  //! Constructor
  Target();

  //! Destructor
  ~Target();

  //! initializer
  bool init(const std::string relative_path);

  //! This method detects the target in a given picture
  //! \param[in] cam_descriptors The desciptors of keypoints in camera picture
  //! \param[in] cam_keypoints The coordinates of keypoints in camera picture
  //! \param[out] good_matches The good matches between the target picture and the camera picture,
  //! in OpenCV format
  //! \param[out] idxs_to_remove A list of indexes of keypoints on the target
  //! \param[out] target_coord The coordinates if the target is detected
  //! \param[in] pose (#ifdef DEBUG_PROJECTION) Pose of the drone estimated with
  //! \param[in] image_cam (#ifdef DEBUG_TARGET) image matrix (OpenCV format)
  //! \return true if the target is detected
  bool detect(cv::Mat cam_descriptors, std::vector< cv::KeyPoint >& cam_keypoints,
              std::vector< cv::DMatch >& good_matches, std::vector< int >& idxs_to_remove,
              std::vector< cv::Point2f >& target_coord
#ifdef DEBUG_PROJECTION
              ,
              boris_drone::Pose3D pose
#endif
#ifdef DEBUG_TARGET
              ,
              cv::Mat& image_cam
#endif
              );

  //! This method draws a green frame to indicate the detected target
  //! \param[in] cam_img image matrix (OpenCV format)
  void draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
            std::vector< cv::DMatch > good_matches, cv::Mat& img_matches);

  //! This method computes the position of the target on the camera image
  void position(std::vector< cv::KeyPoint > cam_keypoints, std::vector< cv::DMatch > good_matches,
                std::vector< cv::Point2f >& coord);
};

bool customLess(cv::DMatch a, cv::DMatch b);

#endif /* boris_drone_TARGET_DETECTION_H */
