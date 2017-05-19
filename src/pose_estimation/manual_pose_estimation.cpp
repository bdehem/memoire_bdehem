/*!
 *  This file is part of boris_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include "boris_drone/manual_pose_estimation.h"

#include <math.h> /* isfinite*/

// Constructor
ManualPoseEstimator::ManualPoseEstimator()
{
  pose_channel = nh.resolveName("pose_estimation");
  pose_pub = nh.advertise< boris_drone::Pose3D >(pose_channel, 1);

  manual_pose_channel = nh.resolveName("manual_pose_estimation");
  manual_pose_pub     = nh.advertise< boris_drone::Pose3D >(manual_pose_channel, 1);
  manual_pose_sub     = nh.subscribe(manual_pose_channel, 1, &ManualPoseEstimator::manualPoseCb, this);

  ros::Publisher  manual_pose_pub;            //!< Publisher of fused pose estimation
  std::string     manual_pose_channel;
  ros::Subscriber manual_pose_sub;

  pose.x = 0;
  pose.y = 0;
  pose.z = 0;
}

// Destructor
ManualPoseEstimator::~ManualPoseEstimator()
{
}


void ManualPoseEstimator::publish_pose()
{
  pose_pub.publish(pose);
}


void ManualPoseEstimator::manualPoseCb(const boris_drone::Pose3D::ConstPtr manualPosePtr)
{
  pose = *manualPosePtr;
  ROS_INFO("Received manual pose.x = %f; y = %f; z = %f",pose.x, pose.y, pose.z);
}


int main(int argc, char** argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
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
