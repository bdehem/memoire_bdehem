/*!
 *  \file strategy.h
 *  \place to select the global strategy
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - the strategy node
 */

#ifndef ucl_drone_STRATEGY_H
#define ucl_drone_STRATEGY_H

// Header files
#include <ros/ros.h>
#include <signal.h>

#include <std_msgs/Float32.h>

#include <ucl_drone/profiling.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/TargetDetected.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>
#include <ucl_drone/constants/strategy_constants.h>


class Strategy
{
private:
  ros::NodeHandle nh;

  //Subscribers
  std::string pose_channel;
  std::string navdata_channel;
  std::string manual_destination_channel;
  ros::Subscriber pose_sub;
  ros::Subscriber navdata_sub;
  ros::Subscriber manual_destination_sub;

  //Publishers
  std::string strategy_channel;
  std::string destination_channel;
  std::string takeoff_channel;
  std::string land_channel;
  ros::Publisher strategy_pub;
  ros::Publisher destination_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;


  //Callbacks
  void manualDestinationCb(const ucl_drone::Pose3D::ConstPtr manualDestinationPtr);
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);
  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);

public:
  //! Constructor & Destructor
  Strategy();
  ~Strategy();

  //Public fields (more convenient to keep them public)
  int strategy;
  int state;
  bool emergency_stop;
  ucl_drone::Pose3D pose;
  ucl_drone::Pose3D destination;

  //Publish
  void publishTakeoff();
  void publishLand();
  void publishDestination();
  void publishStrategy(int strat_type);

  void setDestination(double x, double y, double z, double rotZ);

};

#endif /*ucl_drone_STRATEGY_H */
