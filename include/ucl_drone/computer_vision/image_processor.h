/*!
 *  \file image_processor.h
 *  \brief header file for the main class in the computer vision node.
 *  \author Arnaud Jacques, Alexandre Leclere, Boris Dehem
 *  \date 2016, 2017
 *
 *  Part of ucl_drone. Computer vision ROS node. Contains:
 *        - topics definitions
 *        - subscribers/publishers
 *        - all the calls to other classes
 */

#ifndef ucl_drone_IMAGE_PROCESSOR_H
#define ucl_drone_IMAGE_PROCESSOR_H

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
#include <ucl_drone/computer_vision/target.h>
#include <ucl_drone/computer_vision/processed_image.h>
#include <ucl_drone/constants/feature_types.h>

#include <ucl_drone/read_from_launch.h>


/** \class  ImageProcessor
 * Class of the image processor node for ROS.
 * Creates a ProcessedImage object every time an image is received from the video input.
 * Keypoint extraction happens in the ProcessedImage constructor.
 */
class ImageProcessor
{
private:
  ros::NodeHandle nh;                    //!< Node handle (used by ROS)
  image_transport::ImageTransport it;

  // Subscribers
  std::string     video_channel;             //!< Channel for input video
  image_transport::Subscriber image_sub;     //!< Subscriber to input video
  std::string     pose_channel;              //!< Channel for estimated pose
  ros::Subscriber pose_sub;                  //!< Subscriber to estimated pose
  std::string     reset_pose_channel;        //!< Channel for pose reset
  ros::Subscriber reset_pose_sub;            //!< Subscriber to pose reset
  std::string     end_reset_pose_channel;    //!< Channel for end of pose reset
  ros::Subscriber end_reset_pose_sub;        //!< Subscriber to end of pose reset

  // Publishers
  std::string processed_image_channel_out; //!< Channel for processed images
  ros::Publisher processed_image_pub;      //!< Publisher of processed images

  ros::Time last_full_detection;   //!< Time when we last used detection on entire image
  ros::Time last_hybrid_detection; //!< Time when we last used hybrid detection/tracking

  ProcessedImage* prev_cam_img; //!< the last image processed


  bool use_OpticalFlowPyrLK; //!< launch parameter: if true, processed_image has to use OpticalFlowPyrLK
  bool target_loaded;        //!< true if the target is successfully loaded
  bool pending_reset;        //!< true during a reset

  Target target; //<! the target to detect (this object wraps all needed procedures)

  void setCamChannel(std::string cam_type);

  //! \brief Callback when image is received
  void imageCb(const sensor_msgs::Image::ConstPtr& msg);

  //! \brief Callback when pose is received
  void poseCb(const ucl_drone::Pose3D::ConstPtr& posePtr);

  //! \brief Callback when a pose reset begining is received
  void resetPoseCb(const std_msgs::Empty& msg);

  //! \brief Callback when the pose reset end is received
  void endResetPoseCb(const std_msgs::Empty& msg);

  //! \brief Callback when a navdata is received
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);

  ucl_drone::Pose3D::ConstPtr lastPoseReceived; //!< the last Pose3D message received
  ardrone_autonomy::Navdata lastNavdataReceived;  //!< the last Navdata message received
  sensor_msgs::Image::ConstPtr lastImageReceived; //!< the last Image message received

public:
  //! \brief Contructor.
  ImageProcessor();

  //! \brief Destructor.
  ~ImageProcessor();

/**
 * Publish Processed image.
 * Function to build and send messages with all computed keypoints and target detected.
 */
  void publishProcessedImg();

  bool pose_publishing;   //!< true after receiving the first pose3D message from pose_estimation
  bool video_publishing;  //!< true after receiving the first Image message from ardrone_autonomy
};

#endif /*ucl_drone_IMAGE_PROCESSOR_H*/
