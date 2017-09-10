/*!
 *  \file frame.h
 *  \brief Header file wich defines the Frame class
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef ucl_drone_FRAME_H
#define ucl_drone_FRAME_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/ucl_drone.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/visualization/point_cloud_geometry_handlers.h>  // used here ?

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>                   // pcl::PointXYZRGB
#include <pcl/visualization/cloud_viewer.h>    // used here ?
#include <pcl/visualization/pcl_visualizer.h>  // used here ?
#include <pcl_conversions/pcl_conversions.h>   // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>               // pcl::PointCloud

/* Messages */
#include <sensor_msgs/image_encodings.h>

/* ucl_drone */
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/constants/feature_types.h>

// class MappingNode is defined in ucl_drone/map/simple_map.h
// not declared here because ucl_drone/map/frame.h (current file)
// is also included in ucl_drone/map/simple_map.h or in mapping_node.h

/** \struct Frame
 *  A Frame object stores ImageProcessed message
 */
struct Frame
{
public:
  //! Contructor for an empty object.
  Frame();

  //! Contructor: msg is used for the initialization.
  Frame(ucl_drone::ProcessedImageMsg::ConstPtr msg);

  //! Destructor.
  ~Frame();

  // Attributes
  std::vector<cv::Point2f> img_points; //!< 2D coordinates of keypoints in image in OpenCV format
  cv::Mat descriptors;                 //!< descriptors of keypoints in OpenCV format
  ucl_drone::Pose3D pose;            //!< pose from which frame was taken
  sensor_msgs::Image image;            //!< video image in the ROS format after rescaling
};

#endif /* ucl_drone_FRAME_H */
