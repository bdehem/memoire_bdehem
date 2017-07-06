/*!
 *  \file multi_strategy.h
 *  \brief Basic strategy for multi-agent flight. Specify the mission and the
 * drone roles.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone. Multi-Agent strategy ROS node for the specific mission.
 * Contains:
 *        -
 *        -
 *        -
 *        -
 */

#ifndef boris_drone_MULTI_STRATEGY_H
#define boris_drone_MULTI_STRATEGY_H

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// #define USE_PROFILING  // Uncomment this line to display timing print in the standard output
#include <boris_drone/profiling.h>

// messages
#include <boris_drone/DroneRole.h>
#include <boris_drone/DroneRoles.h>
#include <boris_drone/Pose3D.h>

// boris_drone
#include <boris_drone/drone_role.h>
#include <boris_drone/constants/strategy_constants.h>
#include <boris_drone/boris_drone.h>

/*!
 *  \class MultiStrategy
 *  \brief Provide tools to let drones communicate a common strategy
 */
class MultiStrategy
{
private:
  ros::NodeHandle nh_;

  // Subscriber
  ros::Subscriber ready_sub;
  std::string ready_channel;

  // Publishers
  ros::Publisher drones_roles_pub;
  std::string drones_roles_channel;

  //! \brief Callback: check which drone are ready to take part to the mission
  void readyCb(const boris_drone::DroneRole::ConstPtr readyPtr);

  //! Frequently updated list of drone roles
  std::vector< DroneRole > role_list;

public:
  //! \brief Contructor.
  MultiStrategy();

  //! \brief Destructor.
  ~MultiStrategy();

  void init();

  void PublishDroneRole();
};

#endif /* boris_drone_MULTI_STRATEGY_H */
