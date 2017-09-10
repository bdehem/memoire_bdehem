/*!
 *  \file mapping_node.h
 *  \brief This header file defines the mapping node
 *  \author Boris Dehem
 */

#ifndef ucl_drone_MAPPINGNODE_H
#define ucl_drone_MAPPINGNODE_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/profiling.h>

/* Header files */
#include <ucl_drone/ucl_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>              // pcl::PointCloud

/* Boost */
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/* Messages */
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

/* Sparse Bundle Adjustment */
#include <cvsba/cvsba.h>

/* ucl_drone */
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/BundleMsg.h>
#include <ucl_drone/TargetDetected.h>
#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/read_from_launch.h>

#include <ucl_drone/map/map.h>
#include <ucl_drone/map/keyframe.h>
#include <ucl_drone/map/frame.h>

/*!
 * \class MappingNode
 * This object wraps functions to execute the mapping node
 */
class MappingNode
{
private:
  ros::NodeHandle nh; //!< ROS node handle

  /* Subscribers */
  std::string     strategy_channel;        //!< Channel for strategy messages
  ros::Subscriber strategy_sub;            //!< Subscriber to strategy messages
  std::string     processed_image_channel; //!< Channel for processed images
  ros::Subscriber processed_image_sub;     //!< Subscriber to processed images
  std::string     bundled_channel;         //!< Channel for the result of bundle adjustment
  ros::Subscriber bundled_sub;             //!< Subscriber to the result of bundle adjustment
  std::string     mpe_channel;             //!< Channel for manual pose estimation
  ros::Subscriber mpe_sub;                 //!< Subscriber to manual pose estimation
  std::string     reset_pose_channel;      //!< Channel for pose reset
  ros::Subscriber reset_pose_sub;          //!< Subscriber to pose reset
  std::string     end_reset_pose_channel;  //!< Channel for end of pose reset
  ros::Subscriber end_reset_pose_sub;      //!< Subscriber to end of pose reset


  /* Publishers */
  std::string    pose_visual_channel;     //!< Channel for visual pose
  ros::Publisher pose_visual_pub;         //!< Publisher of visual pose
  std::string    pose_correction_channel; //!< Channel for pose correction
  ros::Publisher pose_correction_pub;     //!< Publisher of pose correction
  std::string    target_channel;          //!< Channel for target
  ros::Publisher target_pub;              //!< Publisher of target

  ucl_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;  //!< last ProcessedImage message received

  /* Attributes */
  int  strategy;        //!< current strategy
  bool pending_reset;   //!< true during a reset
  bool target_detected;

  //! Callbacks
  /**
   * Callback for when a ProcesseImageMsh is received.
   * Called at every image frame.
   */
  void processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in);
  void resetPoseCb(const std_msgs::Empty& msg); //!< Callback for when a pose reset message is received
  void endResetPoseCb(const std_msgs::Empty& msg); //!< Callback for when an end pose reset message is received
  void strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr); //!< Callback for when a strategy message is received
  void bundledCb(const ucl_drone::BundleMsg::ConstPtr bundlePtr); //!< Callback for when the output of bundle adjustment is received
  void manualPoseCb(const ucl_drone::Pose3D::ConstPtr posePtr); //!< Callback for when a manual pose is received (from user)

public:
  Map map; //!< Map object containing the Map and most mapping functions
  ucl_drone::Pose3D PnP_pose; //!< Pose computed from PnP (visual pose)
  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer; //!< Object to visualize the pointcloud

  //! Contructor. Initialize an empty map
  MappingNode();

  //! Destructor.
  ~MappingNode();

  /*!
   * This method publishes a message if the last ImageProcessed contains information about the
   * target detection and add the estimated pose of the detected target in world coordinates
   */
  void targetDetectedPublisher();

  /*!
   * This method publishes a message containing the visual pose estimation
   * @param[in] poseFrame Pose published by sensors when the camera picture was received
   * @param[in] posePnP   Pose estimated with visual information and the contents of the map
   */
  void publishPoseVisual(ucl_drone::Pose3D PnP_pose, ucl_drone::Pose3D frame_pose);

};

#endif /* ucl_drone_MAPPINGNODE_H */
