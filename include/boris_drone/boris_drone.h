/*!
 *  \file boris_drone.h
 *  \brief Common definitions for all nodes in the package boris_drone
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 */

#include <ros/package.h>
#include <ros/ros.h>

#ifndef boris_drone_H
#define boris_drone_H

#define PI 3.14159265

//! Room dimensions (in meters) (pathplanning)
#define SIDE 1

template < class T >
std::string to_string(T i)
{
  std::stringstream ss;
  std::string s;
  ss << i;
  s = ss.str();

  return s;
}

#endif /* boris_drone_H */
