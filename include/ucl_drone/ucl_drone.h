/*!
 *  \file ucl_drone.h
 *  \brief Common definitions for all nodes in the package ucl_drone
 *  \author Arnaud Jacques, Alexandre Leclere & Boris Dehem
 *  \date 2016-2017
 */

#include <ros/package.h>
#include <ros/ros.h>

#ifndef ucl_drone_H
#define ucl_drone_H

#define PI 3.14159265

template <class T>
std::string to_string(T i)
{
  std::stringstream ss;
  std::string s;
  ss << i;
  s = ss.str();

  return s;
}

#endif /* ucl_drone_H */
