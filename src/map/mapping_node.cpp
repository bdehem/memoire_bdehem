/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/mapping_node.h>

MappingNode::MappingNode() : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
{
  iter = 0;
  // Subsribers
  strategy_channel        = nh.resolveName("strategy");
  processed_image_channel = nh.resolveName("processed_image");
  reset_pose_channel      = nh.resolveName("reset_pose");
  end_reset_pose_channel  = nh.resolveName("end_reset_pose");
  bundled_channel         = nh.resolveName("bundled");
  mpe_channel             = nh.resolveName("manual_pose_estimation");
  make_keyframe_channel   = nh.resolveName("make_keyframe");
  strategy_sub        = nh.subscribe(strategy_channel,       10, &MappingNode::strategyCb,       this);
  processed_image_sub = nh.subscribe(processed_image_channel, 1, &MappingNode::processedImageCb, this);
  reset_pose_sub      = nh.subscribe(reset_pose_channel,      1, &MappingNode::resetPoseCb,      this);
  end_reset_pose_sub  = nh.subscribe(end_reset_pose_channel,  1, &MappingNode::endResetPoseCb,   this);
  bundled_sub         = nh.subscribe(bundled_channel,         1, &MappingNode::bundledCb,        this);
  mpe_sub             = nh.subscribe(mpe_channel,             1, &MappingNode::manualPoseCb,     this);
  make_keyframe_sub   = nh.subscribe(make_keyframe_channel,   1, &MappingNode::makeKeyframeCb,   this);

  // Publishers
  pose_visual_channel     = nh.resolveName("pose_visual");
  pose_correction_channel = nh.resolveName("pose_visual_correction");
  target_channel          = nh.resolveName("boris_drone/target_detected");
  go_high_channel         = nh.resolveName("go_high");
  pose_visual_pub     = nh.advertise<boris_drone::Pose3D>(pose_visual_channel,     1);
  pose_correction_pub = nh.advertise<boris_drone::Pose3D>(pose_correction_channel, 1);
  target_pub          = nh.advertise<boris_drone::TargetDetected>(target_channel,  1);
  go_high_pub         = nh.advertise<std_msgs::Float32>(go_high_channel,           1);

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

void MappingNode::manualPoseCb(const boris_drone::Pose3D::ConstPtr posePtr)
{
  ROS_INFO("mapping node received manual pose");
  map.setManualPose(*posePtr);
  PnP_pose = *posePtr;
}

void MappingNode::bundledCb(const boris_drone::BundleMsg::ConstPtr bundlePtr)
{
  map.updateBundle(bundlePtr);
}

void MappingNode::strategyCb(const boris_drone::StrategyMsg::ConstPtr strategyPtr)
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

void MappingNode::processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in)
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
    boris_drone::Pose3D frame_pose = current_frame.pose;
    this->publishPoseVisual(PnP_pose, frame_pose);
  }
  iter++;
  //TOC_DISPLAY(mapprocessimage,"process an image in the map");
}

void MappingNode::makeKeyframeCb(const std_msgs::Empty::ConstPtr msg)
{
  map.make_keyframe = true;
}

void MappingNode::showProcImg(const boris_drone::ProcessedImageMsg::ConstPtr pi)
{
  ROS_INFO("Processed Image:");
  ROS_INFO("\tpose: x=%f, y=%f, z = %f",pi->pose.x,pi->pose.y,pi->pose.z);
  ROS_INFO("\timage: height =%u, width=%u",pi->image.height,pi->image.width);
  ROS_INFO("\tnber of keypoints = %lu",pi->keypoints.size());
  for (unsigned i = 0; i < pi->keypoints.size(); ++i)
  {
    ROS_INFO("\tKeypoint %u: x=%f, y=%f, z=%f",i,pi->keypoints[i].point.x,
                                                 pi->keypoints[i].point.y,
                                                 pi->keypoints[i].point.z);
  }
}

void MappingNode::publishPoseVisual(boris_drone::Pose3D PnP_pose, boris_drone::Pose3D frame_pose)
{
  boris_drone::Pose3D pose_correction;
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
    boris_drone::TargetDetected msg;
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


void MappingNode::publishGoHigh(double altitude)
{
  std_msgs::Float32 msg;
  msg.data = altitude;
  go_high_pub.publish(msg);
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
