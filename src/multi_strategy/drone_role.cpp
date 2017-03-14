/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/drone_role.h>

// Role code + list of strings to contain useful topic names
DroneRole::DroneRole(std::string name)
{
  this->name = name;
  this->role = 1;  // default: STAY_IDLE
}

DroneRole::~DroneRole()
{
}

void DroneRole::SetDroneRole(int role)
{
  this->role = role;
  ROS_DEBUG("Role set: %d", role);
}

void DroneRole::SetDroneRole(int role, std::vector< std::string > params)
{
  this->role = role;
  this->parameters = params;
  ROS_DEBUG("Role set: %d", role);
}

void DroneRole::SetDroneRole(int role, std::string param)
{
  std::vector<std::string> params;
  params.push_back(param);
  this->SetDroneRole(role, params);
}

void DroneRole::SetDroneRole(int role, int number_of_params, ...)
{
  va_list vl;
  va_start(vl, number_of_params);

  std::vector<std::string> params;
  params.resize(number_of_params);
  for (int i = 0; i < number_of_params; i++)
  {
    std::string param(va_arg(vl, char*));
    params.push_back(param);
    this->SetDroneRole(role, params);
  }
  va_end(vl);
}

int DroneRole::GetDroneRole()
{
  return role;
}

boris_drone::DroneRole DroneRole::DroneRoleToMsg()
{
  boris_drone::DroneRole msg;
  msg.name = this->name;
  msg.role = this->role;
  msg.params = this->parameters;
  return msg;
}

void DroneRole::MsgToDroneRole()
{
}

boris_drone::DroneRoles DroneRole::DroneRolesToMsg(std::vector<DroneRole> roles)
{
  boris_drone::DroneRoles msg;
  msg.roles.resize(roles.size());
  for (int i = 0; i < roles.size(); i++)
  {
    msg.roles[i] = roles[i].DroneRoleToMsg();
  }
  return msg;
}
