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

/* boris_drone */
#include <boris_drone/Pose3D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/map/frame.h>
#include <boris_drone/map/camera.h>
#include <algorithm>

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

  Camera* camera; //pointer to the camera that saw this keyframe

  std::vector<cv::Point2f> img_points; //!< 2D coordinates of keypoints in image in OpenCV format
  cv::Mat descriptors;                //!< descriptors of keypoints in OpenCV format
  boris_drone::Pose3D pose;           //!< pose from which the keypoints were observed

  int npts, n_mapped_pts;
  //!< ID of the points in the pointcloud (the map). -1 for points not in map, -2 for deleted points
  std::vector<int> point_IDs;
  std::map<int,int> point_indices; //Maps ID to index of mapped points


  //! Constructor
  //! \param[in] map Pointer to the map
  //! \param[in] frame from which the Keyframe is built
  Keyframe(const Frame& frame, Camera* cam);
  Keyframe();


  Keyframe(const Frame& frame, Camera* cam, const boris_drone::Pose3D pose);


  void setAsSeeing(int ptID, int idx_in_kf);

  void getPointsSeen(std::map<int,std::map<int,int> >& points);

  //! Destructor.
  ~Keyframe();
  void print();
  bool removePoint(int ptID);

};
#endif /* boris_drone_KEYFRAME_H */
