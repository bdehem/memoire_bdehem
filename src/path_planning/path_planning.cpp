/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 * This file receives information from Strategy and the Pose_estimation and publishes to the
 * Controller.
 * It tells the controller where the drone must go as function of the strategy and the position of
 * the drone.
 *
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 */

#include "boris_drone/path_planning.h"

PathPlanning::PathPlanning()
{
  // Subscribers
  pose_channel = nh.resolveName("pose_estimation");
  pose_sub     = nh.subscribe(pose_channel, 10, &PathPlanning::poseCb, this);

  strategy_channel = nh.resolveName("strategy");
  strategy_sub     = nh.subscribe(strategy_channel, 10, &PathPlanning::strategyCb, this);

  destination_channel = nh.resolveName("destination");
  destination_sub     = nh.subscribe(destination_channel, 10, &PathPlanning::destinationCb, this);

  // Publishers
  poseref_channel = nh.resolveName("pose_ref");
  poseref_pub     = nh.advertise<boris_drone::Pose3D>(poseref_channel, 1);

  initPose(destination);
  initPose(pose);
}

// Destructor
PathPlanning::~PathPlanning(){}

void PathPlanning::destinationCb(const boris_drone::Pose3D::ConstPtr destinationPtr)
{
  destination = *destinationPtr;
}

void PathPlanning::publishPoseRef()
{
  poseref_pub.publish(destination);
}

void PathPlanning::publishPoseRef(double x_ref, double y_ref, double z_ref, double rotZ_ref)
{
  boris_drone::Pose3D custom_destination;
  custom_destination.x = x_ref;
  custom_destination.y = y_ref;
  custom_destination.z = z_ref;
  custom_destination.rotX = 0.0;
  custom_destination.rotY = 0.0;
  custom_destination.rotZ = rotZ_ref;
  poseref_pub.publish(custom_destination);
}

void PathPlanning::poseCb(const boris_drone::Pose3D::ConstPtr posePtr)
{
  pose = *posePtr;
}

void PathPlanning::strategyCb(const boris_drone::StrategyMsg::ConstPtr strategyPtr)
{
  strategy = strategyPtr->type;
  if (strategy==69)
  {
    initPose(destination);
  }
}


void PathPlanning::initPose(boris_drone::Pose3D& pose_to_init)
{
  pose_to_init.x = 0.0;
  pose_to_init.y = 0.0;
  pose_to_init.z = 0.0;
  pose_to_init.rotX = 0.0;
  pose_to_init.rotY = 0.0;
  pose_to_init.rotZ = 0.0;
}

// Main function, will publish pose_ref with regard to the selected strategy coming from the
// Strategy node.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  PathPlanning myPath;
  ros::Rate r(20);  // Refresh 20 times per second.
  while (myPath.strategy != 42)
  {
    ros::spinOnce();
    r.sleep();
  }
  while (ros::ok())
  {
    TIC(path);
    myPath.publishPoseRef();
    //TOC(path, "path planning");
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
