/*!
 *  \file drone_role.h
 *  \brief This header file  defines the class to handle drone role objects.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone. Multi-Agent strategy object destined to be transmited as
 *  a ROS message.
 */

#ifndef boris_drone_DRONE_ROLES_H
#define boris_drone_DRONE_ROLES_H

#include <stdarg.h>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// messages
#include <boris_drone/DroneRole.h>
#include <boris_drone/DroneRoles.h>
#include <boris_drone/Pose3D.h>

// boris_drone
#include <boris_drone/boris_drone.h>

/*!
 *  \class DroneRole
 *  \brief Class definition to handle drone roles objects.
 */
class DroneRole
{
private:
  // Attributes

  int role;  //!< Identification number of the role (see boris_drone.h for definitions)
  std::vector<std::string> parameters;  //!< A vector of strings to use as parameters
  std::string name;                     //!< Drone name

public:
  //! \brief Contructor.
  //! \param[in] name String containing the name of the drone
  DroneRole(std::string name);

  //! \brief Destructor.
  ~DroneRole();

  void SetDroneRole(int role);
  void SetDroneRole(int role, std::vector<std::string> params);
  void SetDroneRole(int role, std::string param);
  void SetDroneRole(int role, int number_of_params, ...);

  int GetDroneRole();

  boris_drone::DroneRole DroneRoleToMsg();

  void MsgToDroneRole();

  static boris_drone::DroneRoles DroneRolesToMsg(std::vector< DroneRole > roles);
};

#endif /* boris_drone_DRONE_ROLES_H */
