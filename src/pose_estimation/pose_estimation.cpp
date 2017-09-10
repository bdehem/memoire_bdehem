/*!
 *  This file is part of ucl_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include "ucl_drone/pose_estimation.h"

// Constructor
PoseEstimator::PoseEstimator()
{
  // Subsribers
  navdata_channel     = nh.resolveName("ardrone/navdata");  // raw information from Parrot SDK
  odometry_channel    = nh.resolveName("ardrone/odometry"); // info converted by ardrone autonomy
  pose_visual_channel = nh.resolveName("pose_visual"); // pose estimation from the mapping node
  reset_channel       = nh.resolveName("reset_pose");  // signal to reset the pose to (0,0,h)
  navdata_sub     = nh.subscribe(navdata_channel,    10, &PoseEstimator::navdataCb,    this);
  odometry_sub    = nh.subscribe(odometry_channel,   10, &PoseEstimator::odometryCb,   this);
  pose_visual_sub = nh.subscribe(pose_visual_channel, 1, &PoseEstimator::poseVisualCb, this);
  reset_sub       = nh.subscribe(reset_channel,      10, &PoseEstimator::resetCb,      this);

  // Publishers
  pose_channel           = nh.resolveName("pose_estimation");
  end_reset_pose_channel = nh.resolveName("end_reset_pose");
  pose_pub           = nh.advertise<ucl_drone::Pose3D>(pose_channel,       1);
  end_reset_pose_pub = nh.advertise<std_msgs::Empty>(end_reset_pose_channel, 1);

  // Parameters
  ros::param::get("~use_visual_pose",  use_visual_pose);  // if false, only onboard readings are used

  odometry_publishing   = false;
  visual_pose_available = false;
  pending_reset         = false;
}

// Destructor
PoseEstimator::~PoseEstimator(){}

void PoseEstimator::resetCb(const std_msgs::Empty msg)
{
  pending_reset = true;  // toggle reset state when reset signal is emmitted
}

// this function calls the drone to re-caibrate sensors ! do not call this when the drone is flying !
// the drone needs to lie flat on ground
void PoseEstimator::doFlatTrim()
{
  ros::ServiceClient client = nh.serviceClient< std_srvs::Empty >("motherboard1/ardrone/flattrim");
  std_srvs::Empty srv;
  client.call(srv);
}

// method to reset the pose to origin (except the altitude, the altimeter is used)
void PoseEstimator::doReset()
{
  odometry_x = 0;
  odometry_y = 0;
  odometry_rotX = 0;
  odometry_rotY = 0;
  odometry_rotZ = 0;
  odom_time = lastOdometryReceived.header.stamp;

  lastposeVisualReceived = ucl_drone::Pose3D();
  lastposeVisualReceived.rotZ = 0.0;
  visual_pose_available = false;

  rot_Z_offset = lastNavdataReceived.rotZ / 180.0 * PI;
}

void PoseEstimator::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr)
{
  if (pending_reset)
    return;
  lastNavdataReceived = *navdataPtr;
  previous_rotZ = lastRotZ;
  lastRotZ = lastNavdataReceived.rotZ / 180.0 * PI;
}

void PoseEstimator::poseVisualCb(const ucl_drone::Pose3D::ConstPtr poseVisualPtr)
{
  if (pending_reset)
    return;

  // throw away values older than 1 sec
  if (poseVisualPtr->header.stamp < ros::Time::now() - ros::Duration(0.5))
  {
    ROS_DEBUG("VISUAL POSE disregarded in pose_estimation");
    return;
  }

  lastposeVisualReceived = *poseVisualPtr;
  visual_pose_available = true;
}

void PoseEstimator::odometryCb(const nav_msgs::Odometry::ConstPtr odometryPtr)
{
  // to see what exactly is put in Odometry: see in ardrone_autonomy:
  // ardrone_driver.cpp line 698 to 713

  // avoid to publish NAN values
  if (std::isfinite(odometryPtr->twist.twist.linear.x) &&
      std::isfinite(odometryPtr->twist.twist.linear.y))
  {
    odometry_publishing = true;
    lastOdometryReceived = *odometryPtr;
  }
  else
  {
    ROS_DEBUG("Is ardrone_autonomy odometry a stupid NaN ?");
  }
}

void PoseEstimator::publish_end_reset_pose()
{
  std_msgs::Empty msg;
  end_reset_pose_pub.publish(msg);
  pending_reset = false;
}

void PoseEstimator::publish_pose()
{
  ucl_drone::Pose3D pose_msg;
  bool fusion_success = false;

  fusion_success = use_visual_pose ? queuePoseFusion(pose_msg) : poseFusion(pose_msg);
  if (fusion_success) pose_pub.publish(pose_msg); // && pose_pub.getNumSubscribers() > 0)
  ROS_INFO("PnprotZ = %f ; fusedrotZ = %f",lastposeVisualReceived.rotZ*180/PI,pose_msg.rotZ*180/PI);
}

bool PoseEstimator::poseCopy(ucl_drone::Pose3D& pose_msg)
{
  // Stupid recopying of ardrone_autonomy odometry based on the integration of
  // velocity estimation based on Parrot's optical flow (bottom camera).
  // Some small differences:
  //    - the header timestamp gives the current time
  //    - the angles are given in Euler form (not quaternions): can be deduced
  //      from quaternions (can introduce NaN's) or by directly recopying Navdata
  pose_msg.header.stamp = ros::Time().now();  // lastOdometryReceived.header.stamp;
  pose_msg.header.frame_id = lastOdometryReceived.header.frame_id;
  pose_msg.x = lastOdometryReceived.pose.pose.position.x;
  pose_msg.y = lastOdometryReceived.pose.pose.position.y;
  pose_msg.z = lastOdometryReceived.pose.pose.position.z;

  pose_msg.rotX = lastNavdataReceived.rotX / 180.0 * PI;
  pose_msg.rotY = lastNavdataReceived.rotY / 180.0 * PI;
  pose_msg.rotZ = lastNavdataReceived.rotZ / 180.0 * PI;
  pose_msg.xvel    = lastOdometryReceived.twist.twist.linear.x;
  pose_msg.yvel    = lastOdometryReceived.twist.twist.linear.y;
  pose_msg.zvel    = lastOdometryReceived.twist.twist.linear.z;
  pose_msg.rotXvel = lastOdometryReceived.twist.twist.angular.x;
  pose_msg.rotYvel = lastOdometryReceived.twist.twist.angular.y;
  pose_msg.rotZvel = lastOdometryReceived.twist.twist.angular.z;
}

bool PoseEstimator::poseFusion(ucl_drone::Pose3D& pose_msg)
{
  // with our own odometry integrated from optical flow
  // reset taken into account but visual pose not used
  ros::Time previous_odom_time = odom_time;
  odom_time = lastOdometryReceived.header.stamp;
  double delta_t = (odom_time - previous_odom_time).toSec();

  if (delta_t == 0)  // it seems no new odometry was published
  {
    ROS_DEBUG("Is ardrone_autonomy always responding ?");
    return false;
  }
  if (!std::isfinite(delta_t))
  {
    ROS_DEBUG("Something strange hapenned! delta_t is not finite!");
    return false;
  }

  pose_msg.rotX = lastNavdataReceived.rotX / 180.0 * PI;      // no fusion
  pose_msg.rotY = lastNavdataReceived.rotY / 180.0 * PI;      // no fusion
  pose_msg.z    = lastOdometryReceived.pose.pose.position.z;  // no fusion
  if (lastOdometryReceived.pose.pose.position.z == 0)
    pose_msg.z = lastposeVisualReceived.z;  // for experiments (bc alt unavailable when not flying)


  // Formulas recopied from ardrone_autonomy, to compute pose from integration
  // of the velocities based on Parrot's optical flow (bottom camera).
  double xvel =
    (float)(cos(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.x) -   // - 0.0048) -
            sin(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.y));
  double yvel =
    (float)(sin(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.x) +   // - 0.0048) +
            cos(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.y));

  odometry_x    += xvel*delta_t;
  odometry_y    += yvel*delta_t;
  odometry_rotZ += lastRotZ - previous_rotZ;
  while(odometry_rotZ > M_PI)  odometry_rotZ -= 2 * M_PI;
  while(odometry_rotZ < -M_PI) odometry_rotZ += 2 * M_PI;

  pose_msg.x    = odometry_x;
  pose_msg.y    = odometry_y;
  pose_msg.rotZ = odometry_rotZ;

  pose_msg.header.stamp = odom_time;

  pose_msg.xvel    = lastOdometryReceived.twist.twist.linear.x;
  pose_msg.yvel    = lastOdometryReceived.twist.twist.linear.y;
  pose_msg.zvel    = lastOdometryReceived.twist.twist.linear.z;  // not received when drone is not flying !
  pose_msg.rotXvel = lastOdometryReceived.twist.twist.angular.x;  // not received when drone is not flying !
  pose_msg.rotYvel = lastOdometryReceived.twist.twist.angular.y;  // not received when drone is not flying !
  pose_msg.rotZvel = lastOdometryReceived.twist.twist.angular.z;  // not received when drone is not flying !

  return true;
}

bool PoseEstimator::queuePoseFusion(ucl_drone::Pose3D& pose_msg)
{
  // Simple fusion technique explained in our report our own odometry integrated from optical flow
  ros::Time previous_odom_time = odom_time;
  odom_time = lastOdometryReceived.header.stamp;
  double delta_t = (odom_time - previous_odom_time).toSec();

  if (delta_t == 0)  // it seems no new odometry was published
  {
    ROS_DEBUG("Is ardrone_autonomy still responding ?");
    return false;
  }
  if (!std::isfinite(delta_t))
  {
    ROS_DEBUG("Something strange hapenned! delta_t is not finite!");
    return false;
  }

  if (visual_pose_available)
  {
    ROS_DEBUG("VISUAL POSE received in pose_estimation");
    odometry_x = 0;
    odometry_y = 0;
    odometry_rotZ = 0;

    processQueue(queue_dx,    odometry_x);
    processQueue(queue_dy,    odometry_y);
    processQueue(queue_drotZ, odometry_rotZ);

    visual_pose_available = false;
  }

  pose_msg.rotX = lastNavdataReceived.rotX / 180.0 * PI;
  pose_msg.rotY = lastNavdataReceived.rotY / 180.0 * PI;
  pose_msg.z    = lastOdometryReceived.pose.pose.position.z;  // not received when drone not flying !!!
  if (lastOdometryReceived.pose.pose.position.z == 0)
    pose_msg.z = lastposeVisualReceived.z;

  // Formulas recopied from ardrone_autonomy, to compute pose from integration of the velocities
  double dx =
    (float)((cos(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.x) -   // - 0.0048) -
    sin(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.y)) *  // - 0.0158)) *
    delta_t);
  double dy =
    (float)((sin(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.x) +   // - 0.0048) +
    cos(pose_msg.rotZ) * (lastOdometryReceived.twist.twist.linear.y)) *  // - 0.0158)) *
    delta_t);
  double drotZ = lastRotZ - previous_rotZ;

  odometry_x    += dx;
  odometry_y    += dy;
  odometry_rotZ += drotZ;

  pushQueue(queue_dx,    dx);
  pushQueue(queue_dy,    dy);
  pushQueue(queue_drotZ, drotZ);

  pose_msg.x    = lastposeVisualReceived.x    + odometry_x;
  pose_msg.y    = lastposeVisualReceived.y    + odometry_y;
  pose_msg.rotZ = lastposeVisualReceived.rotZ + odometry_rotZ;
  while(pose_msg.rotZ > M_PI)  pose_msg.rotZ -= 2 * M_PI;
  while(pose_msg.rotZ < -M_PI) pose_msg.rotZ += 2 * M_PI;

  pose_msg.header.stamp = odom_time;

  pose_msg.xvel =    lastOdometryReceived.twist.twist.linear.x;
  pose_msg.yvel =    lastOdometryReceived.twist.twist.linear.y;
  pose_msg.zvel =    lastOdometryReceived.twist.twist.linear.z;   // not received when drone not flying !!!
  pose_msg.rotXvel = lastOdometryReceived.twist.twist.angular.x;  // not received when drone not flying !!!
  pose_msg.rotYvel = lastOdometryReceived.twist.twist.angular.y;  // not received when drone not flying !!!
  pose_msg.rotZvel = lastOdometryReceived.twist.twist.angular.z;  // not received when drone not flying !!!

  return true;
}

void PoseEstimator::pushQueue(std::queue< std::pair<ros::Time, double> >& myqueue, double item)
{
  std::pair<ros::Time, double> pair;
  pair.first  = lastOdometryReceived.header.stamp;
  pair.second = item;
  myqueue.push(pair);
  if (myqueue.size() > 200)
  {
    //ROS_WARN("queue overflowing!");
    myqueue.pop();
  }
}

void PoseEstimator::processQueue(std::queue< std::pair<ros::Time, double> >& myqueue, double& result)
{
  bool popped_is_older_than_visual = true; //ignore values older than last visual pose
  while (popped_is_older_than_visual && !myqueue.empty())
  {
    std::pair<ros::Time, double> popped = myqueue.front();
    myqueue.pop();
    popped_is_older_than_visual = popped.first < lastposeVisualReceived.header.stamp;
  }

  //add values more recent than last visual pose
  while (!myqueue.empty())
  {
    std::pair<ros::Time, double> popped = myqueue.front();
    myqueue.pop();
    result += popped.second;
  }
}

int main(int argc, char** argv)
{
  ROS_INFO_STREAM("pose_estimation started!");

  ros::init(argc, argv, "pose_estimation");
  PoseEstimator myPose;
  ros::Rate r(20);

  ROS_DEBUG("pose estimation initialized");
  while (!myPose.odometry_publishing && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  ROS_DEBUG("ardrone_driver publishing: ending hibernate");
  myPose.doFlatTrim();
  myPose.doReset();
  ROS_INFO("reset done");

  while (ros::ok())
  {
    TIC(pose);
    myPose.publish_pose();
    //TOC(pose, "pose");

    if (myPose.pending_reset)
    {
      ros::Time t = ros::Time::now() + ros::Duration(3);
      while (ros::Time::now() < t)
      {
        ros::spinOnce();
        myPose.doReset();
      }
      myPose.publish_end_reset_pose();
    }

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
