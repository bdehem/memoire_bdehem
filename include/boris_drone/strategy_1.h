/*!
 *  \file strategy.h
 *  \place to select the global strategy
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of boris_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - the strategy node
 */

#ifndef boris_drone_STRATEGY_H
#define boris_drone_STRATEGY_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// #define USE_PROFILING
#include <boris_drone/profiling.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <boris_drone/DroneRole.h>
#include <boris_drone/DroneRoles.h>
#include <boris_drone/StrategyMsg.h>
#include <boris_drone/TargetDetected.h>

// boris_drone
#include <boris_drone/boris_drone.h>

class Strategy
{
private:
  std::string drone_name;

  ros::NodeHandle nh;

  ros::Publisher strategy_pub;
  ros::Subscriber target_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber explore_sub;

  std::string strategy_channel;
  std::string target_channel;
  std::string pose_channel;
  std::string explore_channel;

  boris_drone::TargetDetected lastTargetDetected;
  //! Callback when TargetDetected is received

  void targetDetectedCb(const boris_drone::TargetDetected::ConstPtr targetDetectedPtr);
  void exploreCb(const std_msgs::Empty::ConstPtr emptyPtr);
  void poseCb(const boris_drone::Pose3D::ConstPtr posePtr);

  int strategy;

  float xchosen;
  float ychosen;

  float targetX;
  float targetY;
public:
  //! Constructor
  Strategy();
  //! Destructor
  ~Strategy();


  boris_drone::Pose3D pose;
  bool TargetFound;
  bool FinishedExploring;
  void init();
  void publish_strategy();
  void SetStrategy(int strategy);
  int GetStrategy();
  double getAltitude();
  void SetXYChosen(double xchosen, double ychosen);
};

#endif /*boris_drone_STRATEGY_H */
