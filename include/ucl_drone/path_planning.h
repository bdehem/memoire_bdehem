/*!
 *  \file path_planning.h
 *  \Give the reference position to the drone
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple pathplanning method
 */

#ifndef ucl_drone_PATH_PLANNING_H
#define ucl_drone_PATH_PLANNING_H

// Header files
#include <ros/ros.h>
#include <signal.h>

#include <ucl_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/PoseRef.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/ucl_drone.h>
#include <ucl_drone/cellUpdate.h>
#include <ucl_drone/constants/strategy_constants.h>


class PathPlanning
{
private:
  ros::NodeHandle nh;

  // Subscribers
  std::string pose_channel;
  std::string strategy_channel;
  std::string destination_channel;
  ros::Subscriber pose_sub;
  ros::Subscriber strategy_sub;
  ros::Subscriber destination_sub;

  // Publishers
  std::string poseref_channel;
  ros::Publisher poseref_pub;

  //! Callbacks
  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);
  void strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr);
  void destinationCb(const ucl_drone::Pose3D::ConstPtr destinationPtr);

public:
  //! Constructor & Destructor
  PathPlanning();
  ~PathPlanning();

  //Public fields (more convenient to keep them public)
  int strategy;
  ucl_drone::Pose3D pose;
  ucl_drone::Pose3D destination;

  void publishPoseRef();
  void publishPoseRef(double x_ref, double y_ref, double z_ref, double rotZ_ref);

  void initPose(ucl_drone::Pose3D& pose_to_init);
};

#endif /*ucl_drone_PATH_PLANNING_H */
