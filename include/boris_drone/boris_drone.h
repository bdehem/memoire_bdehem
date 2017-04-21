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

//Strategy codes
#define WAIT 0
#define TAKEOFF 1
#define SEEK 2
#define EXPLORE 3
#define BACKTOBASE 4
#define LAND 5
#define GOTO 6

//! Role code definitions (multistrategy)
#define EMERGENCY_STOP 0
#define STAY_IDLE 1
#define GO_TO 2
#define EXPLORE_AND_MAP 3
#define EXPLORE_UNTIL_TARGET 4

//! Room dimensions (in meters) (pathplanning)
#define SIDE 1

/* begin ===FEATURE TYPE=== */
// computer_vision, target_detection, mapping

#define TYPE_SIFT 1
#define TYPE_FAST 2
#define TYPE_SURF 3
#define TYPE_SURF_128 4
#define TYPE_STAR 5
#define TYPE_BRISK 6
#define TYPE_ORB 7
#define TYPE_SURF_GPU 8
#define TYPE_FREAK 9

#define DETECTOR_TYPE TYPE_SURF  //!< Keypoint detector used in computer_vision
// alternatives: TYPE_SIFT,TYPE_FAST,TYPE_SURF,TYPE_ORB

#define EXTRACTOR_TYPE TYPE_SIFT  //!< Keypoint descriptor used in computer_vision and mapping
// TYPE_SIFT,TYPE_SURF,TYPE_BRISK

// DESCRIPTOR_SIZE
#if EXTRACTOR_TYPE == TYPE_SURF
#define DESCRIPTOR_SIZE 64
#elif EXTRACTOR_TYPE == TYPE_ORB
#define DESCRIPTOR_SIZE 32
#else
#define DESCRIPTOR_SIZE 128
#endif

// DIST_THRESHOLD depends in the descriptor type
#if EXTRACTOR_TYPE == TYPE_SIFT
#define DIST_THRESHOLD 250.0  //!< Max distance s.t. two features descriptions are similar
#elif EXTRACTOR_TYPE == TYPE_SURF
#define DIST_THRESHOLD 0.25  //!< Max distance s.t. two features descriptions are similar
#elif EXTRACTOR_TYPE == TYPE_ORB
#define DIST_THRESHOLD 50.0  //!< Max distance s.t. two features descriptions are similar
#else
#define DIST_THRESHOLD 200.0  //!< Max distance s.t. two features descriptions are similar
#endif

/* end   ===FEATURE TYPE=== */

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
