/*!
 *  \file bundle_adjuster.h
 *  \brief File defining the bundle adjuster node
 *  Inspired by Bundle Adjustment example from CERES solver (bundle_adjuster.cc)
 *  \author Boris Dehem
 *  \date 2017
 */

#ifndef ucl_drone_BUNDLEADJUSTER_H
#define ucl_drone_BUNDLEADJUSTER_H

/* Header files */
#include <ucl_drone/ucl_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/contrib/contrib.hpp>

/* Boost */
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/* Messages */
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

/* ucl_drone */
#include <ucl_drone/BundleMsg.h>

#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/read_from_launch.h>

/*!
 * Class that describes a bundle adjustment problem.
 */
class BALProblem {
 public:
  BALProblem(); //!< Empty Constructor

 /**
  * Constructor
  * @param[in] bundlePtr Bundle message containing all data to be adjusted.
  */
  BALProblem(const ucl_drone::BundleMsg::ConstPtr bundlePtr);
  ~BALProblem(); //!< Destructor
  int num_observations() const;       //!< Get number of landmark observations in the problem
  const double* observations() const; //!< Get pointer to the begin of the array containing 2D position of each observation
  double* mutable_keyframes();        //!< Get pointer to the begin of the array containing the keyframe poses to be adjusted
  double* mutable_points();           //!< Get pointer to the begin of the array containing the landmark positions to be adjusted
  double* mutable_keyframe(int i);    //!< Get pointer to the begin of the ith keyframe pose to be adjusted
  double* mutable_point(int i);       //!< Get pointer to the begin of the ith landmark to be adjusted
  double* ref_pose(int i);            //!< Get pointer to the begin of the reference pose of the ith keyframe
  double* mutable_keyframe_for_observation(int i); //!< Get pointer to the begin of the keyframe the ith observation belongs to.
  double* mutable_point_for_observation(int i);    //!< Get pointer to the begin of the landmark observed in the ith observation

  //private:
  bool    fixed_poses_;      //!< If true, only the landmark positions are adjusted, not the keyframes
  int     num_keyframes_;    //!< Number of keyframes in problem
  int     num_points_;       //!< Number of landmarks in problem
  int     num_observations_; //!< Number of observations (correpondences between a landmark and a keyframe)
  int     num_parameters_;   //!< Total number of optimization parameters
  bool*   fixed_kfs_;        //!< If fixed_kgs_[i] is true, then the ith keyframe's position is kept constant
  int*    point_index_;      //!< Index of the point of each observation
  int*    keyframe_index_;   //!< Index of the keyframe of each observation
  double* observations_;     //!< Observations (2D coordinated of all points)
  double* parameters_;       //!< Array containing all optimization parameters (landmark and keyframe positions)
  double* ref_poses_;        //!< Reference pose of each keyframe
  double* cam_rotations_;    //!< Drone to camera rotation of each keyframe
  ucl_drone::BundleMsg::ConstPtr bundleMsgPtr_; //!< Pointer to bundle message
};

/**
 * \class Bundle Adjuster
 * \brief Contains bundle adjustment node.
 * Solves a bundle adjustment problem each time a bundle message is received.
 */
class BundleAdjuster
{
private:
  ros::NodeHandle nh;

  /* Subscribers */
  ros::Subscriber bundle_sub;     //!< Channel for incoming bundle messages containing the problem
  std::string     bundle_channel; //!< Subscriber to incoming bundle messages containing the problem

  /* Publishers */
  ros::Publisher bundled_pub;     //!< Channel for outgoing bundle messages containing the result
  std::string    bundled_channel; //!< Publisher of outgoing bundle messages containing the result

  double tolerance;   //!< Convergence threshold during optimization
  bool   quiet_ba;    //!< If true, no output in terminal during bundle adjustment
  double huber_delta; //!< Parameter of Huber's loss function
 /**
  * Callback for bundle messages.
  */
  void bundleCb(const ucl_drone::BundleMsg::ConstPtr bundlePtr);

public:
  //! Empty Constructor
  BundleAdjuster();
  //! Destructor.
  ~BundleAdjuster();

 /**
  * Publish result of bundle adjustment.
  * @param[in] bal_problem   BALProblem that was solved
  * @param[in] converged     True if optimization converged
  * @param[in] cost_of_point Contribution of each landmark to the objective function
  * @param[in] time_taken    Time taken for bundle adjustment
  * @param[in] n_iter);      Number of solver iterations
  */
  void publishBundle(const BALProblem& bal_problem, bool converged, std::vector<double>& cost_of_point, double time_taken, int n_iter);

 /**
  * Compute contribution of each landmark to the objective function.
  * @param[in]  problem       ceres::Problem that was solved (contains constrains and objective function)
  * @param[in]  bal_problem   BALProblem that was solved (contains parameters and constants)
  * @param[out] cost_of_point Contribution of each landmark to the objective function
  */
  void computeResiduals(ceres::Problem& problem, BALProblem& bal_problem, std::vector<double>& cost_of_point);
};


#endif /* ucl_drone_BUNDLEADJUSTER_H */
