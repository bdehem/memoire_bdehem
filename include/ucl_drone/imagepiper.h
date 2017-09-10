
#ifndef ucl_drone_IMAGEPIPER_H
#define ucl_drone_IMAGEPIPER_H

// Header files
#include <ros/ros.h>
#include <signal.h>

#include <std_msgs/Float32.h>

#include <ucl_drone/profiling.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/TargetDetected.h>
#include <ucl_drone/ProcessedImageMsg.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

class Piper
{
private:
  ros::NodeHandle nh;
  ros::Publisher im_pub;
  ros::Subscriber proc_im_sub;
  std::string proc_im_channel;
  std::string im_channel;
  void processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in);
  void publish_image(sensor_msgs::Image& img);

public:
  //! Constructor
  Piper();
  //! Destructor
  ~Piper();
};

#endif
