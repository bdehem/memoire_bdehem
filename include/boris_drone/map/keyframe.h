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


#include <tf/transform_datatypes.h>

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
#include <boris_drone/Pose3D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/map/frame.h>
#include <boris_drone/map/map_utils.h>
#include <algorithm>

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
  static int ID_counter;
  int ID;

  std::vector<cv::Point2f> img_points; //!< 2D coordinates of keypoints in image in OpenCV format
  cv::Mat descriptors;                //!< descriptors of keypoints in OpenCV format
  boris_drone::Pose3D pose;           //!< pose from which the keypoints were observed

  int npts;
  std::vector<bool> point_is_mapped;
  //!< ID of the points in the pointcloud (the map). -1 for points not in map, -2 for deleted points
  std::vector<int> point_IDs;
  std::map<int,int> point_idx; //Maps ID to index of mapped points


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  //! Constructor
  //! \param[in] map Pointer to the map
  //! \param[in] frame from which the Keyframe is built
  Keyframe(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Frame& frame);

  //! Destructor.
  ~Keyframe();
  void print();
  void removePoint(int ptID);
  void addPoint(int pt_idx, int pt_ID);

};


void match(Keyframe& kf0, Keyframe& kf1, std::vector<cv::Point3d>& points3D,
     std::vector<int>& idx_kf0, std::vector<int>& idx_kf1,
     std::vector<int>& match_ID, std::vector<bool>& point_is_new,
     int& next_point_ID);
void combine(Keyframe& kf0, Keyframe& kf1, int idx_kf0, int idx_kf1, cv::Point3d& pt3d, int& newptID);

#endif /* boris_drone_KEYFRAME_H */
