/*!
 * \file manual_pose_estimation.h
 * Node used for receiving manual pose from user for tests.
 * \author Boris Dehem
 * \date 2017
 *
 */

#ifndef UCL_MANUAL_POSE_ESTIMATION_H
#define UCL_MANUAL_POSE_ESTIMATION_H

#include <ucl_drone/ucl_drone.h>

#include <ucl_drone/profiling.h>

// Header files
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>

// messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ucl_drone/Pose3D.h>

/** \class  ManualPoseEstimator
 * This class defines an object which wraps all methods to perform the pose estimation based on
 * info from user.
 * Only the z, rotX, rotY coordinates are taken from manual pose. The x, y, rotZ coordinates are taken from visual pose.
 * A manual pose can be sent to the drone by using rostopic pub to publish a Pose3D message to manual_pose_channel
 */
class ManualPoseEstimator
{
private:
  ros::NodeHandle nh;                  //!< Node handle (used by ROS)

  // Publishers
  std::string    pose_channel;         //!< Channel for pose estimation
  ros::Publisher pose_pub;             //!< Published for pose estimation

  // Subscribers
  std::string     manual_pose_channel; //!< Channel for manual pose
  ros::Subscriber manual_pose_sub;     //!< Subscriber to manual pose
  std::string     visual_pose_channel; //!< Channel for visual pose
  ros::Subscriber visual_pose_sub;     //!< Subscriber to visual pose

  // Callbacks
  /**
   * Manual pose callback.
   * Called automatically when a message is received by manual_pose_sub.
   * Updates current pose estimation with z, rotX, and rotY from manual pose message
   */
  void manualPoseCb(const ucl_drone::Pose3D::ConstPtr manualPosePtr);

  /**
   * Visual pose callback.
   * Called automatically when a message is received by visual_pose_sub.
   * Updates current pose estimation with x, y, and rotZ from visual pose message
   */
  void visualPoseCb(const ucl_drone::Pose3D::ConstPtr manualPosePtr);


  ucl_drone::Pose3D pose;       //!< Current estimated pose (using visual and manual information)
  bool has_received_visual_pose;  //!< Flag for when first visual pose is received

public:
  //! Constructor
  ManualPoseEstimator();

  //! Destructor
  ~ManualPoseEstimator();

  void publish_pose(); //!< sends message with the estimated pose
};

#endif /* UCL_POSE_ESTIMATION_H */
