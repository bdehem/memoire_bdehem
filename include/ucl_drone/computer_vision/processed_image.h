/*!
 *  \file image_processor.h
 *  \brief Hheader file for the main class in the computer vision node.
 *  \author Arnaud Jacques, Alexandre Leclere, Boris Dehem
 *  \date 2016, 2017
 *
 *  Part of ucl_drone. Computer vision ROS node. Contains:
 *        - image analysis
 *        - keypoints detection & description
 */

#ifndef ucl_drone_PROCESSED_IMAGE_H
#define ucl_drone_PROCESSED_IMAGE_H


#include <ucl_drone/profiling.h>

#include <boost/shared_ptr.hpp>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

#include <ucl_drone/ucl_drone.h>

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
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/KeyPoint.h>

// ucl_drone
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/computer_vision/processed_image.h>
#include <ucl_drone/constants/feature_types.h>
#include <ucl_drone/computer_vision/target.h>


/*!
 *  \class ProcessedImage
 *  \brief Provide tools to process keypoint extraction
 */
class ProcessedImage
{
private:
  static constexpr double border_detec_frac = 0.15; //!< Percentage threshold of border with no inliers
  ucl_drone::ProcessedImageMsg::Ptr msg;  //!< the message to be sent

public:
  cv_bridge::CvImagePtr cv_img;        //!< Image in OpenCV format
  std::vector<cv::KeyPoint> keypoints; //!< vector of keypoints detected
  cv::Mat descriptors;                 //!< the keypoints descripors in opencv format
  sensor_msgs::Image image;            //!< video image in the ROS format after rescaling
  ucl_drone::Pose3D pose;            //!< estimated pose of the drone before the visual estimation

  int n_pts;    //!< the number of keypoints detected in the last image

  //Constructors
  /**
   * Empty constructor.
   */
  ProcessedImage();

 /**
  * Constructor.
  * Create a ProcessImage object from an image message, and extract keypoints
  * @param[in]  msg                 Image to be processed.
  * @param[in]  pose                Estimation of the pose at the moment the image was observed.
  * @param[in]  prev                Pointer to previous ProcessedImage.
  * @param[in]  OF_mode             Optical Flow mode. 1 = Use only tracking. 0 = use hybrid. -1 = Use only detection
  * @param[out] made_full_detection True if full detection (on entire image was used)
  */
  ProcessedImage(const sensor_msgs::Image& msg, const ucl_drone::Pose3D& pose, ProcessedImage& prev, int OF_mode, bool& made_full_detection);


 /**
  * Find keypoints using tracking.
  * Find keypoints present by looking for keypoints from the previous image using an optical flow method
  * @param[out] tracked_descriptors Descriptors of tracked keypoints.
  * @param[out] tracked_keypoints   Tracked keypoints (2D coordinates in this image).
  * @param[in]  prev                Pointer to previous ProcessedImage.
  * @param[out] min_x Leftmost x-position of tracked keypoints
  * @param[out] max_x Rightmost x-position of tracked keypoints
  * @param[out] min_y Highest y-position of tracked keypoints
  * @param[out] max_y Lowest y-position of tracked keypoints
  */
  bool trackKeypoints(cv::Mat& tracked_descriptors, std::vector<cv::KeyPoint>& tracked_keypoints,
    ProcessedImage& prev,  int& min_x, int& max_x, int& min_y, int& max_y);
 /**
  * Find keypoints using detection and descriptor extraction.
  * Detect keypoints in the image using a detector, and then extract a descriptor for these points
  * @param[out] detected_descriptors Descriptors of detected keypoints.
  * @param[out] detected_keypoints   Detected keypoints (2D coordinates in this image).
  * @param[in]  full_detection       If true, disregard mask and perform detection on entire image.
  * @param[in]  mask                 Mask specifying what part of the image to search for detectors.
  */
  void detectKeypoints(cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints, bool full_detection, cv::Mat& mask);

 /**
  * Combine detected and tracked keypoints.
  * Combined keypoints are saved in the ProcessedImage object.
  * @param[in] tracked_descriptors  Descriptors of detected keypoints.
  * @param[in] tracked_keypoints    Detected keypoints (2D coordinates in this image).
  * @param[in] detected_descriptors Descriptors of tracked keypoints.
  * @param[in] detected_keypoints   Tracked keypoints (2D coordinates in this image).
  */
  void combineKeypoints(cv::Mat& tracked_descriptors,  std::vector<cv::KeyPoint>& tracked_keypoints,
                        cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints);


  ~ProcessedImage(); //!< Destructor

  //!< build a message that can be sent to other nodes
  void convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target);
};

#endif /*ucl_drone_PROCESSED_IMAGE_H*/
