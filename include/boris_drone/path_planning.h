/*!
 *  \file path_planning.h
 *  \Give the reference position to the drone
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of boris_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple pathplanning method
 */

#ifndef boris_drone_PATH_PLANNING_H
#define boris_drone_PATH_PLANNING_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// #define USE_PROFILING
#include <boris_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/PoseRef.h>
#include <boris_drone/StrategyMsg.h>
#include <boris_drone/boris_drone.h>
#include <boris_drone/cellUpdate.h>
#include <boris_drone/constants/strategy_constants.h>

#define UNEXPLORED 0
#define BORDER 1
#define EXPLORED 2

class PathPlanning
{
private:
  ros::NodeHandle nh;

  ros::Publisher explore_pub;
  ros::Publisher poseref_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber strategy_sub;

  std::string explore_channel;
  std::string poseref_channel;
  std::string pose_channel;
  std::string strategy_channel;

  //! Callback when pose is received
  void poseCb(const boris_drone::Pose3D::ConstPtr posePtr);
  void strategyCb(const boris_drone::StrategyMsg::ConstPtr strategyPtr);

  int strategy;

  double grid_origin_x;
  double grid_origin_y;
  double height_of_flight;

  int myGrid[SIDE * 10][SIDE * 10];
  int gridSize;
  int nExploredCell;

  float destination_x;
  float destination_y;
  float destination_z;

  boris_drone::Pose3D pose;

  bool gridInitialized;

  double XMax;
  double YMax;
public:
  //! Constructor
  PathPlanning();
  //! Destructor
  ~PathPlanning();
  void publish_poseref(double x_ref, double y_ref, double z_ref, double rotZ_ref, bool takeoff, bool land);
  void publish_poseref(double x_ref, double y_ref, double z_ref, double rotZ_ref);
  void publish_poseref(bool takeoff, bool land);
  void reset();
  void InitializeGrid();
  void UpdateMap(double x, double y);
  void GetFieldOfView(double x, double y, int* iMin, int* iMax, int* jMin, int* jMax);
  int inGrid(int i);
  void advanced_xy_desired(double* x, double* y);
  void CellToXY(int i, int j, double* xfromcell, double* yfromcell);
  void XYToCell(double x, double y, int* i, int* j);
  int sqdistance(int i, int j, int k, int l);
  int getStrategy();
  void wait();
  void takeOff();
  void seek();
  void explore();
  void backToBase();
  void go_to();
  void land();
};

#endif /*boris_drone_PATH_PLANNING_H */
