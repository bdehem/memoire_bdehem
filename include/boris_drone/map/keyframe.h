/*!
 *  \file keyframe.h
 *  \brief This header file contains Keyframe class definition
 *         In this file, 3D points refer to points in world coordinates,
 *         and 2D points refer to points in the calibrated image coordinates.
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef boris_drone_KEYFRAME_H
#define boris_drone_KEYFRAME_H
#define PCL_NO_PRECOMPILE

#include <boris_drone/boris_drone.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>      // pcl::PointXYZRGB
#include <pcl_ros/point_cloud.h>  // pcl::PointCloud

#include <cvsba/cvsba.h>

#include <boost/shared_ptr.hpp>

/* boris_drone */
#include <boris_drone/PointXYZRGBSIFT.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/opencv_utils.h>

//forward reference to be able to have a pointer to map
class Map;

/**
 * \class Keyframe
 * A Keyframe object stores 3D points seen on a same picture
 * In the future, with triangulation enabled for 3D reconstruction
 * a Keyframe object will store 3D points seen from some close poses
 * (close to be defined according to the camera observation model)
 * A list of Frames (containing 2D keypoint) will be necessary
 */
class Keyframe
{
public:
  cv::Mat descriptors;       //!< descriptors in opencv format

  std::vector<int> points;   //!< indices of points in the pointcloud (the map).
  boris_drone::Pose3D pose;  //!< pose of the drone from which the keypoints were observed

  std::vector<cv::Point2f> unmapped_imgPoints;  //!< 2D coordinates of unmapped keypoints in OpenCV format
  std::vector<cv::Point2f> mapped_imgPoints;    //!< 2D coordinates of mapepd keypoints in OpenCV format

  Map* map;

  //! Constructor.
  //! \param[in] map Pointer to the map
  Keyframe(Map* p_global_map);

  //! Constructor
  //! \param[in] map Pointer to the map
  //! \param[in] pose Pose of the drone from which the keypoints were observed
  Keyframe(Map* p_global_map, boris_drone::Pose3D& pose);

  //! Constructor
  //! \param[in] map Pointer to the map
  //! \param[in] frame from which the Keyframe is built
  Keyframe(Map* p_global_map, const Frame& frame);

  //! Constructor
  //! \param[in] map Pointer to the map
  //! \param[in] pose Pose of the drone from which the keypoints were observed
  //! \param[in] frame from which the Keyframe is built
  Keyframe(Map* p_global_map, const boris_drone::Pose3D& pose, const Frame& frame);

  //! Destructor.
  ~Keyframe();

  //! Method to compare the current Keyframe to a 2D Frame
  //! \param[in] frame Frame to compare
  //! \param[out] idx_matching_points Vector of pairs of indexes: (i,j) with i the map index, j the frame index
  //! \param[out] keyframe_matching_points Vector of matching points (3D) in the Keyframe
  //! \param[out] frame_matching_points Vector of matching points (2D) in the Frame
  void matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                      std::vector< cv::Point3f >& keyframe_matching_points,
                      std::vector< cv::Point2f >& frame_matching_points);
};

#endif /* boris_drone_KEYFRAME_H */
