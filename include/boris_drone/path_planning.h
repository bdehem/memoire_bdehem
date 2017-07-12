/*!
 *  \file path_planning.h
 *  \Give the reference position to the drone
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of boris_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple pathplanning method
 */

#ifndef boris_drone_PATH_PLANNING_H
#define boris_drone_PATH_PLANNING_H

// Header files
#include <ros/ros.h>
#include <signal.h>

#include <boris_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/PoseRef.h>
#include <boris_drone/StrategyMsg.h>
#include <boris_drone/boris_drone.h>
#include <boris_drone/cellUpdate.h>
#include <boris_drone/constants/strategy_constants.h>


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
  void poseCb(const boris_drone::Pose3D::ConstPtr posePtr);
  void strategyCb(const boris_drone::StrategyMsg::ConstPtr strategyPtr);
  void destinationCb(const boris_drone::Pose3D::ConstPtr destinationPtr);

public:
  //! Constructor & Destructor
  PathPlanning();
  ~PathPlanning();

  //Public fields (more convenient to keep them public)
  int strategy;
  boris_drone::Pose3D pose;
  boris_drone::Pose3D destination;

  void publishPoseRef();
  void publishPoseRef(double x_ref, double y_ref, double z_ref, double rotZ_ref);

  void initPose(boris_drone::Pose3D& pose_to_init);
};

#endif /*boris_drone_PATH_PLANNING_H */
