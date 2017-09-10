/*!
 *  \file controller.h
 *  \brief Controller for the ardrone: performs position control
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple altitude Controller
 *  \todo PID
 */

#ifndef ucl_drone_CONTROLLER_H
#define ucl_drone_CONTROLLER_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

#include <ucl_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/PoseRef.h>
#include <ucl_drone/StrategyMsg.h>

// services
#include <std_srvs/Empty.h>
//#include <ucl_drone/SetPoseRef.h>

//! \class BasicController
//! \brief A basic position controller
class BasicController
{
private:
  ros::NodeHandle nh;

  std::string pose_channel;
  std::string poseref_channel;
  std::string strategy_channel;
  ros::Subscriber pose_sub;
  ros::Subscriber poseref_sub;
  ros::Subscriber strategy_sub;


  std::string control_channel;
  std::string toggleState_channel;
  std::string reset_channel;
  ros::Publisher vel_pub;
  ros::Publisher toggleState_pub;
  ros::Publisher reset_pub;


  //! is true if the controller is running
  bool isControlling;

  //! Reference
  float alt_ref;
  double x_ref;
  double y_ref;
  double yaw_ref;

  double Kp_xy;
  double Ki_xy;
  double Kd_xy;
  double Kp_z;
  double Ki_z;
  double Kd_z;
  double Kp_yaw;
  double Ki_yaw;
  double Kd_yaw;

  double integral_x_error;
  double integral_y_error;
  double integral_z_error;
  double integral_yaw_error;
  double anti_windup_yaw;

  double time_xy_old;
  double time_z_old;
  double time_yaw_old;

  double delta_x_old;
  double delta_y_old;
  double delta_z_old;
  double delta_yaw_old;

  double x_desired_old;
  double y_desired_old;
  double z_desired_old;
  double yaw_desired_old;
  double x_vel_cmd_old;
  double y_vel_cmd_old;
  double z_vel_cmd_old;
  double yaw_vel_cmd_old;


  //! Service to set a new Pose
  // ros::ServiceServer setPoseRef_;  //! \todo better if it was a ROS action ? (provide feedback)

  void reguXY(double& xvel_cmd, double& yvel_cmd, double x_mes, double y_mes, double x_desired,
              double y_desired, double yaw, double regu_new_time_xy);
  void reguZ(double& zvel_cmd, double alt_mes, double alt_desired, double regu_new_time_z);
  void reguYaw(double& yawvel_cmd, double yaw_mes, double yaw_desired, double regu_new_time);

  void sendVelToDrone(double pitch, double roll, double yaw_vel, double zvel_cmd, bool force = false);

  //! Callback when pose is received
  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);

  //! Callback when new pose ref is received
  void poseRefCb(const ucl_drone::Pose3D::ConstPtr poseRefPtr);

  void strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr);


  //! Service method
  // bool setPoseRef(ucl_drone::SetPoseRef::Request &, ucl_drone::SetPoseRef::Response &);

public:
  //! Measure
  ucl_drone::Pose3D lastPoseReceived;

  //! Pose desired
  ucl_drone::Pose3D lastPoseRefReceived;

  //! Contructor.
  BasicController();

  //! Destructor.
  ~BasicController();

  void init();

  void controlLoop();

};

#endif /* ucl_drone_CONTROLLER_H */
