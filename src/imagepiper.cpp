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

#include "boris_drone/imagepiper.h"

// Constructor
Piper::Piper()
{
  // Subscribers
  proc_im_channel = nh.resolveName("/processed_image");
  proc_im_sub = nh.subscribe(proc_im_channel, 10, &Piper::processedImageCb, this);

  // Publishers
  im_channel = nh.resolveName("/ardrone/front/camera/image_raw");
  im_pub = nh.advertise<sensor_msgs::Image>(im_channel, 1);
}

// Destructor
Piper::~Piper()
{
}

void Piper::publish_image(sensor_msgs::Image& img)
{
  im_pub.publish(img);
}

void Piper::processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  sensor_msgs::Image img = processed_image_in->image;
  publish_image(img);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "piper");
  Piper myPiper;
  ros::Rate r(20);  // This function refreshes every 1/20 second.
  ros::spinOnce();
  ros::Duration(10).sleep();
  r.sleep();
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
