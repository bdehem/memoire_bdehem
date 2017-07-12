/*!
 *  \file pose_estimation.h
 *  \brief Pose estimation node of the drone (x,y,z,theta,phi,psi) in an absolute coordinate frame.
 * At the present: on the basis of the Odometry received from ardrone_autonomy and Visual pose
 * estimation from the mapping node In future developpment: Kalman filters, visual odometry, raw
 * sensors, etc.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone. Pose estimation ROS node for the
 *  ardrone.
 */

#ifndef UCL_MANUAL_POSE_ESTIMATION_H
#define UCL_MANUAL_POSE_ESTIMATION_H

#include <boris_drone/boris_drone.h>

#include <boris_drone/profiling.h>

// Header files
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>

// messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <boris_drone/Pose3D.h>

/** \class  PoseEstimator
 *  This class defines an object which wraps all method to perform the pose estimation based on
 * fusion between sensors and visual information
 */
class ManualPoseEstimator
{
private:
  ros::NodeHandle nh;

  // Publishers:
  ros::Publisher pose_pub;            //!< Publisher of fused pose estimation
  std::string    pose_channel;

  ros::Publisher  manual_pose_pub;            //!< Publisher of fused pose estimation
  std::string     manual_pose_channel;
  ros::Subscriber manual_pose_sub;
  std::string     visual_pose_channel;
  ros::Subscriber visual_pose_sub;

  void manualPoseCb(const boris_drone::Pose3D::ConstPtr manualPosePtr);
  void visualPoseCb(const boris_drone::Pose3D::ConstPtr manualPosePtr);

  //! Copy of the last visual pose computed in map node
  boris_drone::Pose3D pose;
  bool has_received_visual_pose;

public:
  //! Constructor
  ManualPoseEstimator();

  //! Destructor
  ~ManualPoseEstimator();

  void publish_pose();            //!< sends message with the fused pose
};

#endif /* UCL_POSE_ESTIMATION_H */
