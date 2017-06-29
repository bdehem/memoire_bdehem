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

#include <boris_drone/computer_vision/computer_vision.h>
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
