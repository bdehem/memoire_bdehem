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

#include <boris_drone/computer_vision/computer_vision.h>

/*!
 *  \class ProcessedImage
 *  \brief Provide tools to process keypoint extraction
 */
class ProcessedImage
{
private:
  static constexpr double border_detec_frac = 0.1;
  int n_pts;    //! the number of keypoints detected in the last image
  boris_drone::ProcessedImageMsg::Ptr msg;  //! the message to be sent

public:
  cv_bridge::CvImagePtr cv_img;           //! Image in OpenCV format
  std::vector<cv::KeyPoint> keypoints;  //! vector of keypoints detected
  cv::Mat descriptors;                    //! the keypoints descripors in opencv format
  sensor_msgs::Image image;               //! video image in the ROS format after rescaling
  boris_drone::Pose3D pose;  //! estimated pose of the drone before the visual estimation

  bool testing;
  //constructors
  ProcessedImage();
  //ProcessedImage(const sensor_msgs::Image image, const boris_drone::Pose3D pose, ProcessedImage& prev,
  //               bool use_OpticalFlowPyrLK);
  //ProcessedImage(const sensor_msgs::Image msg, const boris_drone::Pose3D pose_); //no optical flow
  ProcessedImage(const sensor_msgs::Image msg, const boris_drone::Pose3D pose_, ProcessedImage& prev);
  void trackKeypoints(cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints, ProcessedImage& prev);
  void detectKeypoints(cv::Mat& descriptors, std::vector<cv::KeyPoint>& keypoints, bool full_detection);
  void combineKeypoints(cv::Mat& tracked_descriptors,  std::vector<cv::KeyPoint>& tracked_keypoints,
                                   cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints);


  ~ProcessedImage();                          //! Destructor

  //! build a message that can be sent to other nodes
  void convertToMsg(boris_drone::ProcessedImageMsg::Ptr& msg, Target target);
};

#endif /*boris_drone_PROCESSED_IMAGE_H*/
