/*!
 *  \file image_processor.h
 *  \brief Hheader file for the main class in the computer vision node.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone. Computer vision ROS node. Contains:
 *        - image analysis
 *        - keypoints detection & description
 */

#ifndef boris_drone_PROCESSED_IMAGE_H
#define boris_drone_PROCESSED_IMAGE_H


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

// messages
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/ProcessedImageMsg.h>
#include <boris_drone/KeyPoint.h>

// boris_drone
#include <boris_drone/opencv_utils.h>
#include <boris_drone/map/projection_2D.h>
#include <boris_drone/computer_vision/processed_image.h>
#include <boris_drone/constants/feature_types.h>
#include <boris_drone/computer_vision/target.h>


/*!
 *  \class ProcessedImage
 *  \brief Provide tools to process keypoint extraction
 */
class ProcessedImage
{
private:
  static constexpr double border_detec_frac = 0.15;
  static int last_number_of_keypoints;
  boris_drone::ProcessedImageMsg::Ptr msg;  //! the message to be sent

public:
  cv_bridge::CvImagePtr cv_img;           //! Image in OpenCV format
  std::vector<cv::KeyPoint> keypoints;  //! vector of keypoints detected
  cv::Mat descriptors;                    //! the keypoints descripors in opencv format
  sensor_msgs::Image image;               //! video image in the ROS format after rescaling
  boris_drone::Pose3D pose;  //! estimated pose of the drone before the visual estimation

  int n_pts;    //! the number of keypoints detected in the last image
  bool testing;
  //constructors
  ProcessedImage();
  ProcessedImage(const sensor_msgs::Image& msg, const boris_drone::Pose3D& pose, ProcessedImage& prev, int OF_mode, bool& made_full_detection);


  bool trackKeypoints(cv::Mat& tracked_descriptors, std::vector<cv::KeyPoint>& tracked_keypoints,
    ProcessedImage& prev,  int& min_x, int& max_x, int& min_y, int& max_y);

  void detectKeypoints(cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints, bool full_detection, cv::Mat& mask);
  void combineKeypoints(cv::Mat& tracked_descriptors,  std::vector<cv::KeyPoint>& tracked_keypoints,
                                   cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints);


  ~ProcessedImage();                          //! Destructor

  //! build a message that can be sent to other nodes
  void convertToMsg(boris_drone::ProcessedImageMsg::Ptr& msg, Target target);
};

#endif /*boris_drone_PROCESSED_IMAGE_H*/
