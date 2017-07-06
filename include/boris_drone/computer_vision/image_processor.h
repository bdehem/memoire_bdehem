/*!
 *  \file image_processor.h
 *  \brief Hheader file for the main class in the computer vision node.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone. Computer vision ROS node. Contains:
 *        - topics definitions
 *        - subscribers/publishers
 *        - all the call to other classes
 */

#ifndef boris_drone_IMAGE_PROCESSOR_H
#define boris_drone_IMAGE_PROCESSOR_H

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
#include <boris_drone/computer_vision/target.h>
#include <boris_drone/computer_vision/processed_image.h>
#include <boris_drone/constants/feature_types.h>

#include <boris_drone/read_from_launch.h>

/*  \class ImageProcessor
 *  \brief Class of the image processor node for ROS.
 */
class ImageProcessor
{
private:
  ros::NodeHandle nh_;

  // Subscribers
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  std::string video_channel_;
  ros::Subscriber pose_sub;
  std::string pose_channel;
  ros::Subscriber reset_pose_sub;
  std::string reset_pose_channel;
  ros::Subscriber end_reset_pose_sub;
  std::string end_reset_pose_channel;

  // Publishers
  ros::Publisher processed_image_pub;
  std::string processed_image_channel_out;

  //! the last image processed
  ProcessedImage* prev_cam_img;

  //! launch parameter: if true, processed_image has to use OpticalFlowPyrLK
  bool use_OpticalFlowPyrLK;

  //! true if the target is successfully loaded
  bool target_loaded;

  //! true during a reset
  bool pending_reset;

  //! the target to detect (this object wraps all needed procedures)
  Target target;

  //! \brief Callback when image is received
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

  //! \brief Callback when pose is received
  void poseCb(const boris_drone::Pose3D::ConstPtr& posePtr);

  //! \brief Callback when a pose reset begining is received
  void resetPoseCb(const std_msgs::Empty& msg);

  //! \brief Callback when the pose reset end is received
  void endResetPoseCb(const std_msgs::Empty& msg);

  //! \brief Callback when a naavdata is received
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);

  boris_drone::Pose3D::ConstPtr lastPoseReceived;    //! the last Pose3D message received
  ardrone_autonomy::Navdata lastNavdataReceived;   //! the last Navdata message received
  sensor_msgs::Image::ConstPtr lastImageReceived;  //! the last Image message received

public:
  //! \brief Contructor.
  ImageProcessor();

  //! \brief Destructor.
  ~ImageProcessor();

  void publishProcessedImg();  //! function to build and send messages with all computed keypoints
                               //! and target detected

  bool pose_publishing;   //! true after receiving the first pose3D message from pose_estimation
  bool video_publishing;  //! true after receiving the first Image message from ardrone_autonomy
};

#endif /*boris_drone_IMAGE_PROCESSOR_H*/
