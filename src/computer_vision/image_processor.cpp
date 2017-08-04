/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/computer_vision/image_processor.h>

ImageProcessor::ImageProcessor() : it(nh)
{
  std::string cam_type;       // bottom or front
  std::string video_channel_;  // path to the undistorted video channel

  cv::initModule_nonfree();  // initialize the opencv module which contains SIFT and SURF

  // Get all parameters from the launch file
  bool autonomy_unavailable = false;  // true if the ardrone_autonomy node is not launched
  ros::param::get("~autonomy_unavailable", autonomy_unavailable);
  ros::param::get("~use_OpticalFlowPyrLK", this->use_OpticalFlowPyrLK);
  ros::param::get("~cam_type", cam_type);
  if      (cam_type == "front")  video_channel_ = "ardrone/front/image_raw";
  else if (cam_type == "bottom") video_channel_ = "ardrone/bottom/image_raw";
  ros::param::get("~video_channel", video_channel_);

  // Read camera calibration coefficients in the launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  // Subscribers
  video_channel          = nh.resolveName(video_channel_);
  pose_channel           = nh.resolveName("pose_estimation");
  reset_pose_channel     = nh.resolveName("reset_pose");
  end_reset_pose_channel = nh.resolveName("end_reset_pose");
  image_sub_         = it.subscribe(video_channel,          1, &ImageProcessor::imageCb,        this);
  pose_sub           = nh.subscribe(pose_channel,           1, &ImageProcessor::poseCb,         this);
  reset_pose_sub     = nh.subscribe(reset_pose_channel,     1, &ImageProcessor::resetPoseCb,    this);
  end_reset_pose_sub = nh.subscribe(end_reset_pose_channel, 1, &ImageProcessor::endResetPoseCb, this);

  // Initialize publisher of processed_image
  processed_image_channel_out = nh.resolveName("processed_image");
  processed_image_pub = nh.advertise<boris_drone::ProcessedImageMsg>(processed_image_channel_out, 1);

  if (!autonomy_unavailable)  // then set the drone to the selected camera
  {

  }


  // Initialize an empty processed_image
  this->prev_cam_img = new ProcessedImage();
  #ifdef DEBUG_TARGET
    cv::namedWindow(OPENCV_WINDOW);
  #endif /* DEBUG_TARGET */

  // Load and initialize the target object
  target_loaded = target.init(TARGET_RELPATH);

  pose_publishing  = false;
  video_publishing = false;
  pending_reset    = false;
  last_full_detection   = ros::Time::now() - ros::Duration(100.0);
  last_hybrid_detection = ros::Time::now() - ros::Duration(100.0);
}

void ImageProcessor::setCamChannel(std::string cam_type)
{
  ros::ServiceClient client = nh.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");
  ardrone_autonomy::CamSelect srv;
  if      (cam_type == "bottom") srv.request.channel = 1;
  else if (cam_type == "front")  srv.request.channel = 0;
  else ROS_ERROR("invalid cam type");
  int counter_call_success = 0;
  ros::Rate r(1);
  while (counter_call_success < 2)
  {
    r.sleep();
    if (client.call(srv))
    {
      ROS_INFO("Camera toggled ?");
      if (srv.response.result) counter_call_success += 1;
    }
    else
    {
      ROS_INFO("Failed to call service setcamchannel, try it again in 1sec...");
    }
  }
}

ImageProcessor::~ImageProcessor()
{
#ifdef DEBUG_TARGET
  cv::destroyWindow(OPENCV_WINDOW);
#endif /* DEBUG_TARGET */
  delete this->prev_cam_img;
}

void ImageProcessor::resetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = true;
  this->prev_cam_img = new ProcessedImage();
}

void ImageProcessor::endResetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = false;
}

/* This function is called every time a new image is published */
void ImageProcessor::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::imageCb");
  lastImageReceived = msg;
  video_publishing = true;
}

/* This function is called every time a new pose is published */
void ImageProcessor::poseCb(const boris_drone::Pose3D::ConstPtr& posePtr)
{
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::poseCb");
  lastPoseReceived = posePtr;
  pose_publishing = true;
}

/* This function is called at every loop of the current node */
void ImageProcessor::publishProcessedImg()
{
  TIC(imageprocessor);
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::publishProcessedImg");

  // give all data to process the last image received (keypoints and target detection)
  //ProcessedImage cam_img(*lastImageReceived, *lastPoseReceived, *prev_cam_img, use_OpticalFlowPyrLK);
  int OF_mode;
  if (ros::Time::now() - last_full_detection > ros::Duration(9.0)||!prev_cam_img->cv_img||prev_cam_img->n_pts < 20)
  {
    OF_mode = -1;
    last_full_detection   = ros::Time::now();
    last_hybrid_detection = ros::Time::now();
  }
  else if (ros::Time::now() - last_hybrid_detection > ros::Duration(3.0))
  {
    OF_mode = 0;
    last_hybrid_detection = ros::Time::now();
  }
  else OF_mode = 1;

  bool test = false;
  ProcessedImage cam_img(*lastImageReceived, *lastPoseReceived, *prev_cam_img, OF_mode, test);
  if (test&&OF_mode!=-1) ROS_WARN("anomaly");

  // initialize the message to send
  boris_drone::ProcessedImageMsg::Ptr msg(new boris_drone::ProcessedImageMsg);
  // build the message to send
  cam_img.convertToMsg(msg, target);
  //TOC(processed_image, "processedImage");

  TIC(publish);
  // publish the message
  processed_image_pub.publish(msg);
  //TOC(publish, "publisher");

  // replace the previous image processed
  delete this->prev_cam_img;
  this->prev_cam_img = new ProcessedImage(cam_img);
  //if (OF_mode == -1)
    //TOC_DISPLAY(imageprocessor,"detection");
  //if (OF_mode == 0)
    //TOC_DISPLAY(imageprocessor,"hybrid   ");
  //if (OF_mode == 1)
    //TOC_DISPLAY(imageprocessor,"tracking ");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "computer_vision");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  // initialize the node object
  ImageProcessor ic;

  // rate of this node
  //ros::Rate r(6);  // 12Hz =  average frequency at which we receive images
  //ros::Rate r(15);  // 12Hz =  average frequency at which we receive images
  ros::Rate r(20);

  // wait until pose and video are available
  while ((!ic.pose_publishing || !ic.video_publishing) && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  while (ros::ok())  // while the node is not killed by Ctrl-C
  {
    ros::spinOnce();
    ic.publishProcessedImg();
    r.sleep();
  }

  return 0;
}
