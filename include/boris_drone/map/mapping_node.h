/*!
 *  \file mapping_node.h
 *  \brief This header file defines classes for the mapping node and visual pose estimation
 * (keyframe-based)
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef boris_drone_SIMPLE_MAP_H
#define boris_drone_SIMPLE_MAP_H
#define PCL_NO_PRECOMPILE

#define USE_PROFILING
#include <boris_drone/profiling.h>

/* Header files */
#include <boris_drone/boris_drone.h>

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

/* boris_drone */
#include <boris_drone/Pose3D.h>
#include <boris_drone/ProcessedImageMsg.h>
#include <boris_drone/StrategyMsg.h>
#include <boris_drone/BundleMsg.h>
#include <boris_drone/TargetDetected.h>
#include <boris_drone/map/projection_2D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/read_from_launch.h>

#include <boris_drone/map/map.h>
#include <boris_drone/map/keyframe.h>
#include <boris_drone/map/frame.h>

/*!
 * \class MappingNode
 * This object wraps functions to execute the mapping node
 */
class MappingNode
{
private:
  ros::NodeHandle nh;

  /* Subscribers */
  ros::Subscriber processed_image_sub;
  std::string     processed_image_channel;
  ros::Subscriber reset_pose_sub;
  std::string     reset_pose_channel;
  ros::Subscriber end_reset_pose_sub;
  std::string     end_reset_pose_channel;
  ros::Subscriber strategy_sub;
  std::string     strategy_channel;
  ros::Subscriber bundled_sub;
  std::string     bundled_channel;

  /* Publishers */
  ros::Publisher pose_visual_pub;
  std::string    pose_visual_channel;
  ros::Publisher pose_correction_pub;
  std::string    pose_correction_channel;
  ros::Publisher target_pub;
  std::string    target_channel;
  ros::Publisher go_high_pub;
  std::string    go_high_channel;
  int iter;

  boris_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;  //!< s last message received

  /* Attributes */
  int  strategy;       //!< current strategy
  bool pending_reset;  //!< true during a reset
  bool target_detected;

  //! Callbacks
  void processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in);
  void resetPoseCb(const std_msgs::Empty& msg);
  void endResetPoseCb(const std_msgs::Empty& msg);
  void strategyCb(const boris_drone::StrategyMsg::ConstPtr strategyPtr);
  void bundledCb(const boris_drone::BundleMsg::ConstPtr bundlePtr);


  void showProcImg(const boris_drone::ProcessedImageMsg::ConstPtr pi);

public:
  Map map;

  //! The visualizer object to perform projection
  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

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
  void publishPoseVisual(boris_drone::Pose3D PnP_pose, boris_drone::Pose3D frame_pose);

  void publishGoHigh(double altitude);
};

#endif /* boris_drone_SIMPLE_MAP_H */
