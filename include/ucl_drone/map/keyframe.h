/*!
 *  \file keyframe.h
 *  \brief This header file contains Keyframe class definition
 *  \authors Boris Dehem
 *  \year 2017
 */

#ifndef ucl_drone_KEYFRAME_H
#define ucl_drone_KEYFRAME_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/ucl_drone.h>


#include <tf/transform_datatypes.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* ucl_drone */
#include <ucl_drone/constants/feature_types.h>

#include <ucl_drone/Pose3D.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/map/frame.h>
#include <ucl_drone/map/camera.h>
#include <algorithm>

/**
 * \class Keyframe
 * A keyframe is a snapshot of a previous pose of the drone, with the image seen by the drone at that time.
 */
class Keyframe
{
public:
  static int ID_counter; //<! Static counter to give a unique ID to each keyframe
  int ID;                //<! Unique ID of this keyframe

  Camera* camera; //<! pointer to the camera that saw this keyframe

  std::vector<cv::Point2f> img_points; //!< 2D coordinates of keypoints in the image
  cv::Mat descriptors;                 //!< descriptors of the keypoints
  ucl_drone::Pose3D pose;            //!< estimated pose (corrected)
  ucl_drone::Pose3D ref_pose;        //!< estimated pose at keyframe creation time

  int npts;         //!< Number of observed keypoints
  int n_mapped_pts; //!< Number of observed keypoints that have been mapped (landmarks)

  std::vector<int> point_IDs;      //!< IDs of the observed landmarks. -1 for points not in map, -2 for deleted points
  std::map<int,int> point_indices; //!< Map giving the index withing this keyframe of landmarks from their ID

  Keyframe(); //!< Empty Constructor

 /** Constructor
  * \param[in] frame Frame from which the Keyframe is built
  * \param[in] cam   Camera seeing this keyframe
  */
  Keyframe(const Frame& frame, Camera* cam);

 /** Set keypoint at index idx_in_kf as corresponding to landmark ptID.
  */
  void setAsSeeing(int ptID, int idx_in_kf);

 /**
  * Get all landmarks observed by this keyframe
  * @param[in,out] points Map with as keys the IDs of landmarks seen, and as value, another map. The inner map has as key the ID of the keyframe and as value the index of the points in this keyframe corresponding to the landmark.
  */
  void getPointsSeen(std::map<int,std::map<int,int> >& points);

  //! Destructor.
  ~Keyframe();

 /**
  * Display information about this keyframe in the terminal
  */
  void print();

 /**
  * Set point ptID as not seen by this keyframe
  */
  bool removePoint(int ptID);

};
#endif /* ucl_drone_KEYFRAME_H */
