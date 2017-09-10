/*!
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 * This file receives information from path_planning and pose_estimation and publishes mainly in
 * cmd_vel to communicate with the drone.
 * It controls the position of the drone.
 *
 *  \authors Julien Gerardy & Felicien Schiltz
 *  \date 2016
 *
 */

#include "ucl_drone/controller.h"

static bool urgency_signal = false;  // true when Ctrl-C (Emergency stop)

// Constructor
BasicController::BasicController()
{
  // Subscribers
  pose_channel     = nh.resolveName("pose_estimation");
  poseref_channel  = nh.resolveName("pose_ref");
  strategy_channel = nh.resolveName("strategy");
  pose_sub     = nh.subscribe(pose_channel,     10, &BasicController::poseCb,     this);
  poseref_sub  = nh.subscribe(poseref_channel,  10, &BasicController::poseRefCb,  this);
  strategy_sub = nh.subscribe(strategy_channel, 10, &BasicController::strategyCb, this);

  // Publishers
  control_channel     = nh.resolveName("cmd_vel");
  toggleState_channel = nh.resolveName("ardrone/reset");
  reset_channel       = nh.resolveName("reset_pose");
  vel_pub         = nh.advertise<geometry_msgs::Twist>(control_channel, 1);
  toggleState_pub = nh.advertise<std_msgs::Empty>(toggleState_channel,  1, true);
  reset_pub       = nh.advertise<std_msgs::Empty>(reset_channel,        1, true);

  //PID gains
  ros::param::get("~Kp_xy", Kp_xy);
  ros::param::get("~Ki_xy", Ki_xy);
  ros::param::get("~Kd_xy", Kd_xy);

  ros::param::get("~Kp_z", Kp_z);
  ros::param::get("~Ki_z", Ki_z);
  ros::param::get("~Kd_z", Kd_z);

  ros::param::get("~Kp_yaw", Kp_yaw);
  ros::param::get("~Ki_yaw", Ki_yaw);
  ros::param::get("~Kd_yaw", Kd_yaw);

  ros::param::get("~anti_windup_yaw", anti_windup_yaw);

  // Initialization of some variables useful for regulation (see below).
  isControlling = false;
  integral_x_error   = 0;
  integral_y_error   = 0;
  integral_z_error   = 0;
  integral_yaw_error = 0;
  x_desired_old   = 0;
  y_desired_old   = 0;
  z_desired_old   = 0;
  yaw_desired_old = 0;
}

BasicController::~BasicController(){}

// Regulation in the XY plane. The X and Y regulator are exactly the same. It is important to know
// that the drone is controlled by Forward-Backward, Left-Right motion in its own repere. So we need
// to use a rotation matrix to match drone command and good drone movement.

void BasicController::reguXY(double& x_vel_cmd, double& y_vel_cmd, double x_current, double y_current,
                             double x_desired, double y_desired, double yaw,
                             double time_xy)
{
  // printf("reguXY x_desired : %lf y_desired : %lf \n", x_desired, y_desired);
  double p_y, i_y, d_y, p_x, i_x, d_x;
  double x_vel_cmd_abs, y_vel_cmd_abs;
  double time_difference = (time_xy - time_xy_old);
  double delta_x  = x_desired - x_current;
  double delta_y  = y_desired - y_current;
  double delta_xy = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

  // If navdata has the same timestamp, send the last command (in order to avoid null division)
  if (time_difference == 0)
  {
    x_vel_cmd = x_vel_cmd_old;
    y_vel_cmd = y_vel_cmd_old;
    return;
  }

  if (x_desired_old != x_desired ||  y_desired_old != y_desired)  // If pose_ref has changed, integral error is reset.
  {
    integral_x_error = 0;
    integral_y_error = 0;
  }
  integral_x_error += delta_x * time_difference;
  integral_y_error += delta_y * time_difference;

  // axis transformation (from absolute axis to drone axis in order to give it velocities
  // commands)
  double c_theta = cos(yaw);
  double s_theta = sin(yaw);


  p_x = delta_x;
  p_y = delta_y;

  d_x = (delta_x - delta_x_old) / time_difference;
  d_y = (delta_y - delta_y_old) / time_difference;

  i_x = integral_x_error;
  i_y = integral_y_error;

  //x and y commands in the world frame
  x_vel_cmd_abs = (Kp_xy*p_x + Ki_xy*i_x + Kd_xy*d_x);
  y_vel_cmd_abs = (Kp_xy*p_y + Ki_xy*i_y + Kd_xy*d_y);

  //transformation to drone frame
  x_vel_cmd = x_vel_cmd_abs * c_theta - y_vel_cmd_abs * s_theta;
  y_vel_cmd = x_vel_cmd_abs * s_theta + y_vel_cmd_abs * c_theta;

  // Velocities command in the drone repere.
  x_vel_cmd_old = x_vel_cmd;
  y_vel_cmd_old = y_vel_cmd;
  delta_x_old  = delta_x;
  delta_y_old  = delta_y;
  time_xy_old  = time_xy;
}

// Regulation in altitude, according to the Z axis.
void BasicController::reguZ(double& z_vel_cmd, double z_current, double z_desired, double time_z)
{
  double p, i, d;
  double time_difference = (time_z - time_z_old);
  double delta_z = z_desired - z_current;

  // If navdata has the same timestamp, send the last command
  if (time_difference == 0)
  {
    z_vel_cmd = z_vel_cmd_old;
    return;
  }

  if (z_desired_old != z_desired)  // Reset of the integral term if target has changed
  {
    integral_z_error = 0;
  }
  integral_z_error += delta_z * time_difference;

  p = delta_z;
  i = integral_z_error;
  d = (delta_z - delta_z_old) / time_difference;

  // Z velocity command sent to the drone
  z_vel_cmd = (Kp_z*p + i*Ki_z + Kd_z*d);
  if (z_vel_cmd > 0) z_vel_cmd*=2.5;

  z_vel_cmd_old = z_vel_cmd;
  z_desired_old = z_desired;
  delta_z_old   = delta_z;
  time_z_old    = time_z;
}

// Regulation of theta, the yaw angle.
void BasicController::reguYaw(double& yaw_vel_cmd, double yaw_current, double yaw_desired, double time_yaw)
{
  double p, i, d;

  double time_difference = (time_yaw - time_yaw_old);
  double delta_yaw = yaw_desired - yaw_current;

  // If navdata has the same timestamp, send the last command
  if (time_difference == 0)
  {
    yaw_vel_cmd = yaw_vel_cmd_old;
    return;
  }

  if (yaw_desired_old != yaw_desired)  integral_yaw_error = 0;

  // The yaw angle is included in [-pi;pi] interval.
  if (delta_yaw > 3.14159)        delta_yaw -= 2 * 3.14159;
  else if (delta_yaw < -3.14159)  delta_yaw += 2 * 3.14159;
  integral_yaw_error += delta_yaw * time_difference;

  p = delta_yaw;
  i = integral_yaw_error;
  d = (delta_yaw - delta_yaw_old) / time_difference;


  // Velocity sent to the drone.
  yaw_vel_cmd = (Kp_yaw*p + Ki_yaw*i  + Kd_yaw*d);
  // rotational speed limitation (wrongly called anti_windup).
  if (yaw_vel_cmd >  anti_windup_yaw)
  {
    integral_yaw_error = (anti_windup_yaw - (Kp_yaw*p + Kd_yaw*d))/Ki_yaw;
    yaw_vel_cmd =  anti_windup_yaw;
  }
  if (yaw_vel_cmd < -anti_windup_yaw)
  {
    integral_yaw_error = (-anti_windup_yaw - (Kp_yaw*p + Kd_yaw*d))/Ki_yaw;
    yaw_vel_cmd = -anti_windup_yaw;
  }

  //ROS_INFO_THROTTLE(1,"....................");
  //ROS_INFO_THROTTLE(1,"current time = % 7.5f", time_yaw);
  //ROS_INFO_THROTTLE(1,"last yaw     = % 7.5f", yaw_current);
  //ROS_INFO_THROTTLE(1,"last yaw ref = % 7.5f", yaw_desired);
  //ROS_INFO_THROTTLE(1,"command (p)  = % 7.5f", Kp_yaw * p);
  //ROS_INFO_THROTTLE(1,"command (i)  = % 7.5f", i * Ki_yaw);
  //ROS_INFO_THROTTLE(1,"command (d)  = % 7.5f", Kd_yaw * d);
  //ROS_INFO_THROTTLE(1,"command      = % 7.5f", yaw_vel_cmd);

  yaw_vel_cmd_old = yaw_vel_cmd;
  yaw_desired_old = yaw_desired;
  delta_yaw_old   = delta_yaw;
  time_yaw_old    = time_yaw;
}

// This function publishes the computed velocities in the topic to the drone.
void BasicController::sendVelToDrone(double pitch, double roll, double yaw_vel, double z_vel_cmd,
                                     bool force /*=false*/)
{
  geometry_msgs::Twist cmdT;
  cmdT.linear.x  = pitch;
  cmdT.linear.y  = roll;
  cmdT.angular.z = yaw_vel;
  cmdT.linear.z  = z_vel_cmd;

  // printf("cmdT.angular.z : %lf \n", cmdT.angular.z);
  // assume that while actively controlling,
  // the above for will never be equal to zero, so i will never hover.
  cmdT.angular.x = cmdT.angular.y = 0;  // TODO: good idea ?

  if (isControlling || force)
  {
    vel_pub.publish(cmdT);
  }
}

// This function is called when this node receives a message from the topic "pose_estimation". So it
// takes this message and put it in a variable where it will be used in the other functions.
void BasicController::poseCb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  lastPoseReceived = *posePtr;
}

// This function is called when this node receives a message from the topic "poseref". So it
// takes this message and put it in a variable where it will be used in the other functions.
void BasicController::poseRefCb(const ucl_drone::Pose3D::ConstPtr poseRefPtr)
{
  if (lastPoseRefReceived.x    != poseRefPtr->x)    integral_x_error   = 0.0;
  if (lastPoseRefReceived.y    != poseRefPtr->y)    integral_y_error   = 0.0;
  if (lastPoseRefReceived.z    != poseRefPtr->z)    integral_z_error   = 0.0;
  if (lastPoseRefReceived.rotZ != poseRefPtr->rotZ) integral_yaw_error = 0.0;
  lastPoseRefReceived = *poseRefPtr;
}

void BasicController::strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr)
{
  if (strategyPtr->type==69)
  {
    sendVelToDrone(0,0,0,0,true);
    isControlling = false;
  }
  else if (strategyPtr->type==42&&!isControlling)
  {
    integral_x_error   = 0.0;
    integral_y_error   = 0.0;
    integral_z_error   = 0.0;
    integral_yaw_error = 0.0;
    isControlling = true;
  }
}

void BasicController::controlLoop()
{
  double x_vel_cmd, y_vel_cmd, z_vel_cmd, yaw_vel_cmd;
  double current_time = lastPoseReceived.header.stamp.sec + lastPoseReceived.header.stamp.nsec / pow(10, 9);

  //ROS_INFO_THROTTLE(2,"pose received at controller: x = % 4.2f, rotX = % 7.1f", lastPoseReceived.x, lastPoseReceived.rotX*180/PI);
  //ROS_INFO_THROTTLE(2,"                           : y = % 4.2f, rotY = % 7.1f", lastPoseReceived.y, lastPoseReceived.rotY*180/PI);
  //ROS_INFO_THROTTLE(2,"                           : z = % 4.2f, rotZ = % 7.1f", lastPoseReceived.z, lastPoseReceived.rotZ*180/PI);
  //ROS_INFO_THROTTLE(2,"                                                    ");


  reguXY(x_vel_cmd, y_vel_cmd, lastPoseReceived.x, lastPoseReceived.y, lastPoseRefReceived.x,
         lastPoseRefReceived.y, lastPoseReceived.rotZ, current_time);

  reguZ(z_vel_cmd, lastPoseReceived.z, lastPoseRefReceived.z, current_time);
  reguYaw(yaw_vel_cmd, lastPoseReceived.rotZ, lastPoseRefReceived.rotZ, current_time);

  sendVelToDrone(x_vel_cmd, y_vel_cmd, yaw_vel_cmd, z_vel_cmd, false);
}


// Main function, launching the controLoop function.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");
  BasicController bc;
  ROS_INFO("controller started!");
  while (ros::ok())
  {
    //ROS_INFO_THROTTLE(1,"estimated pose: x=% 4.2f; y=% 4.2f; z=% 4.2f; rotZ=% 4.2f",bc.lastPoseReceived.x, bc.lastPoseReceived.y, bc.lastPoseReceived.z, bc.lastPoseReceived.rotZ);
    //ROS_INFO_THROTTLE(1,"  desired pose: x=% 4.2f; y=% 4.2f; z=% 4.2f; rotZ=% 4.2f",bc.lastPoseRefReceived.x, bc.lastPoseRefReceived.y, bc.lastPoseRefReceived.z, bc.lastPoseRefReceived.rotZ);
    TIC(control);
    bc.controlLoop();
    //TOC(control, "control");
    ros::spinOnce();  // if we dont want this we have to place callback and services in threads
  }
  return 0;
}
