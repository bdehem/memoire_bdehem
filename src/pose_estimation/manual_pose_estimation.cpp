/*!
 *  This file is part of ucl_drone 2017.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Boris Dehem
 *  \date 2017
 */

#include "ucl_drone/manual_pose_estimation.h"

#include <math.h> /* isfinite*/

// Constructor
ManualPoseEstimator::ManualPoseEstimator()
{
  pose_channel = nh.resolveName("pose_estimation");
  pose_pub = nh.advertise<ucl_drone::Pose3D>(pose_channel, 1);

  manual_pose_channel = nh.resolveName("manual_pose_estimation");
  manual_pose_sub     = nh.subscribe(manual_pose_channel, 1, &ManualPoseEstimator::manualPoseCb, this);

  visual_pose_channel = nh.resolveName("pose_visual");
  visual_pose_sub     = nh.subscribe(visual_pose_channel, 1, &ManualPoseEstimator::visualPoseCb, this);

  std::string     manual_pose_channel;
  ros::Subscriber manual_pose_sub;

  has_received_visual_pose = false;
  pose.x = 0;
  pose.y = 0;
  pose.z = 0;
  pose.rotX = 0;
  pose.rotY = 0;
  pose.rotZ = 0;
}

// Destructor
ManualPoseEstimator::~ManualPoseEstimator()
{
}

void ManualPoseEstimator::publish_pose()
{
  pose.header.stamp = ros::Time::now();
  pose_pub.publish(pose);
}

void ManualPoseEstimator::manualPoseCb(const ucl_drone::Pose3D::ConstPtr manualPosePtr)
{
  if (!has_received_visual_pose)
    pose = *manualPosePtr;
  else
  {
    pose.z = manualPosePtr->z;
    pose.rotX = manualPosePtr->rotX;
    pose.rotY = manualPosePtr->rotY;
  }
  ROS_INFO("Received manual pose.x = %f; y = %f; z = %f",manualPosePtr->x, manualPosePtr->y, manualPosePtr->z);
}

void ManualPoseEstimator::visualPoseCb(const ucl_drone::Pose3D::ConstPtr visualPosePtr)
{
  //return;
  has_received_visual_pose = true;
  pose.x = visualPosePtr->x;
  pose.y = visualPosePtr->y;
  pose.rotZ = visualPosePtr->rotZ;
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("pose_estimation started!");

  ros::init(argc, argv, "pose_estimation");
  ManualPoseEstimator myPose;
  ros::Rate r(20);

  while (ros::ok())
  {
    myPose.publish_pose();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
