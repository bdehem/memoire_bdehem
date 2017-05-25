
#ifndef boris_drone_BUNDLEADJUSTER_H
#define boris_drone_BUNDLEADJUSTER_H

/* Header files */
#include <boris_drone/boris_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

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

/* boris_drone */
#include <boris_drone/BundleMsg.h>

#include <boris_drone/opencv_utils.h>
#include <boris_drone/read_from_launch.h>


class BALProblem {
 public:
  BALProblem();
  BALProblem(const boris_drone::BundleMsg::ConstPtr bundlePtr);
  ~BALProblem();
  int num_observations() const;
  const double* observations() const;
  double* mutable_cameras();
  double* mutable_points();
  double* mutable_camera(int i);
  double* mutable_point(int i);
  double* mutable_camera_for_observation(int i);
  double* mutable_point_for_observation(int i);
  //private:
  bool fixed_poses_;
  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;
  int* point_index_;
  int* camera_index_;
  double* observations_;
  double* parameters_;
};

/*!
 * \class Bundle Adjuster
 */
class BundleAdjuster
{
private:
  ros::NodeHandle nh;
  /* Subscribers */
  ros::Subscriber bundle_sub;
  std::string     bundle_channel;
  /* Publishers */
  ros::Publisher bundled_pub;
  std::string    bundled_channel;
  void bundleCb(const boris_drone::BundleMsg::ConstPtr bundlePtr);
public:
  //! Contructor. Initialize an empty map
  BundleAdjuster();
  //! Destructor.
  ~BundleAdjuster();
  void publishBundle(const BALProblem& bal_problem);
};


#endif /* boris_drone_BUNDLEADJUSTER_H */
