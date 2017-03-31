/*!
 *  \file frame.h
 *  \brief Header file wich defines the Frame class
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef boris_drone_FRAME_H
#define boris_drone_FRAME_H
#define PCL_NO_PRECOMPILE

#include <boris_drone/boris_drone.h>

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

/* boris_drone */
#include <boris_drone/PointXYZRGBSIFT.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/ProcessedImageMsg.h>
// #include <boris_drone/map/simple_map.h>

// class Map is defined in boris_drone/map/simple_map.h
// not declared here because boris_drone/map/frame.h (current file)
// is also included in boris_drone/map/simple_map.h or in map_keyframe_based.h

/** \class Frame
 *  A Frame object stores ImageProcessed message and posseses facilities to convert
 *  2D points in OpenCV format. There are also facilities to convert 3D points to PCL format.
 *  In the future Frame objects can also be useful for Bundle Adjustment
 */
class Frame
{
private:
  boris_drone::ProcessedImageMsg msg;    //!< ProcessedImage Message
  std::vector<cv::Point2f> imgPoints;    //!< 2D coordinates of keypoints in OpenCV format
  boris_drone::Pose3D pose_visual_msg;   //!< visual pose estimated (not used in this implementation)
  cv::Mat descriptors;                   //!< descriptors of keypoints in OpenCV format

public:
  //! Contructor for an empty object.
  Frame();

  //! Contructor: msg is used for the initialization.
  Frame(boris_drone::ProcessedImageMsg::ConstPtr msg);

  //! Destructor.
  ~Frame();

  //! This function converts points to the PCL format
  //! \param[in] idx_points: indexes (in this->imgPoints) of points to be converted
  //! \param[in] points_out: 3D points in the OpenCV format to be converted in PCL
  //! \param[in] keyframe_id: used to fill in the field which correponds in XYZRGBSIFTPoint
  //! \param[out] pointcloud: result of the conversion to PCL format
  void convertToPcl(std::vector< int >& idx_points, std::vector< cv::Point3f > points_out,
                    int keyframe_id, pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud);

  //! This function converts all points to the PCL format
  //! \param[in] points_out: points to be converted in the OpenCV format
  //! \param[in] keyframe_id: used to fill in the field which correponds in XYZRGBSIFTPoint
  //! \param[out] pointcloud: result of the conversion to PCL format
  void convertToPcl(std::vector< cv::Point3f > points_out, int keyframe_id,
                    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud);
};

#endif /* boris_drone_FRAME_H */
