/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/mapping_node.h>

MappingNode::MappingNode() : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
{
  // Subsribers
  strategy_channel        = nh.resolveName("strategy");
  processed_image_channel = nh.resolveName("processed_image");
  reset_pose_channel      = nh.resolveName("reset_pose");
  end_reset_pose_channel  = nh.resolveName("end_reset_pose");
  bundled_channel         = nh.resolveName("bundled");
  mpe_channel             = nh.resolveName("manual_pose_estimation");
  strategy_sub        = nh.subscribe(strategy_channel,       10, &MappingNode::strategyCb,       this);
  processed_image_sub = nh.subscribe(processed_image_channel, 1, &MappingNode::processedImageCb, this);
  reset_pose_sub      = nh.subscribe(reset_pose_channel,      1, &MappingNode::resetPoseCb,      this);
  end_reset_pose_sub  = nh.subscribe(end_reset_pose_channel,  1, &MappingNode::endResetPoseCb,   this);
  bundled_sub         = nh.subscribe(bundled_channel,         1, &MappingNode::bundledCb,        this);
  mpe_sub             = nh.subscribe(mpe_channel,             1, &MappingNode::manualPoseCb,     this);

  // Publishers
  pose_visual_channel     = nh.resolveName("pose_visual");
  pose_correction_channel = nh.resolveName("pose_visual_correction");
  target_channel          = nh.resolveName("ucl_drone/target_detected");
  pose_visual_pub     = nh.advertise<ucl_drone::Pose3D>(pose_visual_channel,     1);
  pose_correction_pub = nh.advertise<ucl_drone::Pose3D>(pose_correction_channel, 1);
  target_pub          = nh.advertise<ucl_drone::TargetDetected>(target_channel,  1);

  this->target_detected = false;
  this->pending_reset   = false;

  // get camera parameters in launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  map = Map(&nh);

  // initialize the map and the visualizer
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(map.cloud, 0, 255, 0);
  visualizer->setBackgroundColor(0, 0.1, 0.3);
  visualizer->addPointCloud<pcl::PointXYZ>(map.cloud, single_color, "SIFT_cloud");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "SIFT_cloud");
  visualizer->addCoordinateSystem(1.0);  // red: x, green: y, blue: z

  ROS_DEBUG("mapping_node initialized");
  ROS_INFO("these are logger messages:");
  ROS_DEBUG("ROS_DEBUG message");
  ROS_INFO("ROS_INFO message");
  ROS_WARN("ROS_WARN message");
  ROS_ERROR("ROS_ERROR message");
  ROS_FATAL("ROS_FATAL message");
}

MappingNode::~MappingNode()
{
}

void MappingNode::manualPoseCb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  ROS_INFO("mapping node received manual pose");
  map.setManualPose(*posePtr);
  PnP_pose = *posePtr;
}

void MappingNode::bundledCb(const ucl_drone::BundleMsg::ConstPtr bundlePtr)
{
  map.updateBundle(bundlePtr);
}

void MappingNode::strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr)
{
  strategy = strategyPtr->type;
}

void MappingNode::resetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = true;
  processed_image_sub.shutdown();
  map.reset();
  processed_image_sub = nh.subscribe(processed_image_channel, 3, &MappingNode::processedImageCb, this);
  // update visualizer
  this->visualizer->updatePointCloud<pcl::PointXYZ>(map.cloud, "SIFT_cloud");
}

void MappingNode::endResetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = false;
}

void MappingNode::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  TIC(mapprocessimage);
  //ROS_INFO_THROTTLE(1,"pending reset? %d, strategy? %d", pending_reset, strategy);
  bool PnP_success;
  //if (pending_reset || strategy == WAIT || strategy == TAKEOFF)
  //  return;
  target_detected = processed_image_in->target_detected;
  if (target_detected)
    lastProcessedImgReceived = processed_image_in;
  Frame current_frame(processed_image_in);
  PnP_success = map.processFrame(current_frame, PnP_pose);
  this->visualizer->updatePointCloud<pcl::PointXYZ>(map.cloud, "SIFT_cloud");
  if (PnP_success)
  {
    ucl_drone::Pose3D frame_pose = current_frame.pose;
    this->publishPoseVisual(PnP_pose, frame_pose);
  }
  //TOC_DISPLAY(mapprocessimage,"process an image in the map");
}

void MappingNode::publishPoseVisual(ucl_drone::Pose3D PnP_pose, ucl_drone::Pose3D frame_pose)
{
  ucl_drone::Pose3D pose_correction;
  pose_correction.header.stamp = ros::Time::now();
  pose_correction.x = frame_pose.x - PnP_pose.x;
  pose_correction.y = frame_pose.y - PnP_pose.y;
  pose_correction.z = frame_pose.z - PnP_pose.z;
  pose_correction.rotX = frame_pose.rotX - PnP_pose.rotX;
  pose_correction.rotY = frame_pose.rotY - PnP_pose.rotY;
  pose_correction.rotZ = frame_pose.rotZ - PnP_pose.rotZ;
  PnP_pose.header.stamp = ros::Time::now();
  pose_visual_pub.publish(PnP_pose);
  pose_correction_pub.publish(pose_correction);
}



void MappingNode::targetDetectedPublisher()
{
  if (target_detected)
  {
    ucl_drone::TargetDetected msg;
    msg.pose = lastProcessedImgReceived->pose;
    msg.img_point.x = lastProcessedImgReceived->target_points[4].x;
    msg.img_point.y = lastProcessedImgReceived->target_points[4].y;
    msg.img_point.z = 0;
    std::vector< cv::Point2f > target_center(1);
    target_center[0].x = lastProcessedImgReceived->target_points[4].x;
    target_center[0].y = lastProcessedImgReceived->target_points[4].y;
    std::vector< cv::Point3f > world_coord;
    projection_2D(target_center, msg.pose, world_coord);
    msg.world_point.x = world_coord[0].x;
    msg.world_point.y = world_coord[0].y;
    msg.world_point.z = world_coord[0].z;
    target_pub.publish(msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_map");
  ROS_INFO_STREAM("simple map started!");

  MappingNode map_node;
  ros::Time t = ros::Time::now() + ros::Duration(1);
  while (ros::Time::now() < t)
    map_node.visualizer->spinOnce(100);

  //ros::Rate r(3);
  ros::Rate r(20);

  while (ros::ok())
  {
    map_node.visualizer->spinOnce(10);
    map_node.targetDetectedPublisher();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
