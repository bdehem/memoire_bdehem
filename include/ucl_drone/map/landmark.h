/*!
 *  \file landmark.h
 *  \brief This header file contains Landmark class definition
 *  \authors Boris Dehem
 *  \year 2017
 */


#ifndef ucl_drone_LANDMARK_H
#define ucl_drone_LANDMARK_H

#include <opencv2/core/core.hpp>
#include <set>
#include <time.h>
#include <ros/ros.h>

/**
 * \struct Landmark
 * A Landmark is a 3D point with a descriptor used to localize the drone.
 */
struct Landmark
{
private:
  static int ID_counter; //<! Static counter to give a unique ID to each landmark
public:
  std::set<int> keyframes_seeing; //!< Set of IDs of the keyframes seeing this landmark
  cv::Point3d coordinates;        //!< Estimated 3D coordinates of this landmark
  cv::Mat descriptor;             //!< Descriptor of this landmark
  int ID;                         //!< Unique ID of this landmark
  int times_inlier;               //!< Number of times this landmark was an inlier during RANSAC
  int times_outlier;              //!< Number of times this landmark was an outlier during RANSAC
  ros::Time creation_time;        //!< Time when this landmark was created

 /**
  * Constructor
  * @param[in] coordinates 3D coordinates of the landmark
  * @param[in] descriptor  Descriptor of the landmark
  */
  Landmark(cv::Point3d& coordinates, cv::Mat& descriptor);
  Landmark(); //!< Empty Constructor

  //! Destructor.
  ~Landmark();
 /**
  * Replace the estimated 3D coordinates of this landmark by coordinates
  */
  void updateCoords(cv::Point3d& coordinates);

 /**
  * Set this landmark as seen by keyframe with ID kfID
  */
  void setAsSeenBy(int kfID);
 /**
  * Set this landmark as not seen by keyframe with ID kfID
  */
  bool setAsUnseenBy(int kfID);

 /**
  * Display information about this landmark in the terminal
  */
  void print();
};

#endif /* ucl_drone_LANDMARK_H */
