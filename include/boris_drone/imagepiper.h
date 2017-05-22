
#ifndef boris_drone_IMAGEPIPER_H
#define boris_drone_IMAGEPIPER_H

// Header files
#include <ros/ros.h>
#include <signal.h>

#include <std_msgs/Float32.h>

// #define USE_PROFILING
#include <boris_drone/profiling.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <boris_drone/DroneRole.h>
#include <boris_drone/DroneRoles.h>
#include <boris_drone/StrategyMsg.h>
#include <boris_drone/TargetDetected.h>
#include <boris_drone/ProcessedImageMsg.h>

// boris_drone
#include <boris_drone/boris_drone.h>

class Piper
{
private:
  ros::NodeHandle nh;
  ros::Publisher im_pub;
  ros::Subscriber proc_im_sub;
  std::string proc_im_channel;
  std::string im_channel;
  void processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in);
  void publish_image(sensor_msgs::Image& img);

public:
  //! Constructor
  Piper();
  //! Destructor
  ~Piper();
};

#endif
