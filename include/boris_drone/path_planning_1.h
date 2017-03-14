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

  int i;
  int j;
  //! Callback when pose is received
  void poseCb(const boris_drone::Pose3D::ConstPtr posePtr);
  void strategyCb(const std_msgs::Int16::ConstPtr strategyPtr);

  int strategy;

  float home_x;
  float home_y;
  float grid_origin_x;
  float grid_origin_y;

  int myGrid[SIDE * 10][SIDE * 10];
  int gridSize;
  int nExploredCell;

public:
  //! Constructor
  PathPlanning();
  //! Destructor
  ~PathPlanning();


  boris_drone::Pose3D pose;

  boris_drone::cellUpdate cellUpdateMsg;
  bool gridInitialized;
  void init();
  double next_x;
  double next_y;
  double next_z;
  double next_rotZ;
  bool instruction_publishing;
  void publish_poseref(bool takeoff, bool land);
  void yaw_desired();
  bool xy_desired();
  void reset();
  void SetRef(double x_ref, double y_ref, double z_ref, double rotZ_ref);
  void InitializeGrid();
  int bordersList[SIDE * 100];  // What is the maximum number of frontier cells?
  bool ThereIsAWallCell(int i, int j);
  void UpdateMap(double x, double y);
  void GetFieldOfView(double x, double y, int* iMin, int* iMax, int* jMin, int* jMax);
  void advanced_xy_desired();
  void CellToXY(int i, int j, double* xfromcell, double* yfromcell);
  int* XYToCell(double x, double y);
  double sqdistance(int i, int j, int k, int l);
  int getStrategy();
  double XMax;
  double YMax;
  int CellUp;
  int CellDown;
  int CellRight;
  int CellLeft;
  double xfromcell;
  double yfromcell;
  double xfromcell2;
  double yfromcell2;
  double alt;
};

#endif /*boris_drone_PATH_PLANNING_H */
