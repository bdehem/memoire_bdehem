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

#include "boris_drone/strategy.h"

// Constructor
Strategy::Strategy()
{
  // Subscribers
  pose_channel = nh.resolveName("pose_estimation");
  pose_sub     = nh.subscribe(pose_channel, 10, &Strategy::poseCb, this);

  navdata_channel = nh.resolveName("navdata");
  navdata_sub     = nh.subscribe(navdata_channel, 1, &Strategy::navdataCb, this);

  manual_destination_channel = nh.resolveName("manual_destination");
  manual_destination_sub     = nh.subscribe(manual_destination_channel,1,&Strategy::manualDestinationCb,this);

  // Publishers
  strategy_channel = nh.resolveName("strategy");
  strategy_pub = nh.advertise<boris_drone::StrategyMsg>(strategy_channel, 1);

  destination_channel = nh.resolveName("destination");
  destination_pub     = nh.advertise<boris_drone::Pose3D>(destination_channel, 1);

  takeoff_channel = nh.resolveName("ardrone/takeoff");
  takeoff_pub     = nh.advertise<std_msgs::Empty>(takeoff_channel, 1, true);

  land_channel = nh.resolveName("ardrone/land");
  land_pub     = nh.advertise<std_msgs::Empty>(land_channel, 1);

  make_keyframe_channel = nh.resolveName("make_keyframe");
  make_keyframe_pub     = nh.advertise<std_msgs::Empty>(make_keyframe_channel, 1);

  destination.x = 0.0;
  destination.y = 0.0;
  destination.z = 0.0;
  destination.rotX = 0.0;
  destination.rotY = 0.0;
  destination.rotZ = 0.0;
  strategy = WAIT;
  state = 2;
  emergency_stop = false;
}

// Destructor
Strategy::~Strategy()
{
}

void Strategy::publishTakeoff() {  takeoff_pub.publish(std_msgs::Empty()); }
void Strategy::publishLand()    {  land_pub.publish(std_msgs::Empty());    }

void Strategy::publishDestination()
{
  destination_pub.publish(destination);
}

void Strategy::publishStrategy(int strat_type)
{
  boris_drone::StrategyMsg strat;
  strat.type = strat_type;
  strategy_pub.publish(strat);
}

void Strategy::publishMakeKeyframe()
{
    make_keyframe_pub.publish(std_msgs::Empty());
}


// This function is called when the topic of the target_detected of this drone publishes something.
void Strategy::manualDestinationCb(const boris_drone::Pose3D::ConstPtr manualDestinationPtr)
{
  ROS_INFO("received manual destination: x=%f; y=%f; z=%f; rotZ=%f", manualDestinationPtr->x, manualDestinationPtr->y, manualDestinationPtr->z, manualDestinationPtr->rotZ);
  destination = *manualDestinationPtr;
  publishDestination();
}

void Strategy::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr)
{
  state = navdataPtr->state;
}

void Strategy::poseCb(const boris_drone::Pose3D::ConstPtr posePtr)
{
  pose = *posePtr;
}

void Strategy::setDestination(double x, double y, double z, double rotZ)
{
  destination.x = x;
  destination.y = y;
  destination.z = z;
  destination.rotX = 0.0;
  destination.rotY = 0.0;
  destination.rotZ = rotZ;
}


void waitSomeSeconds(double secs, ros::Rate r)
{
  ros::Time t_ref = ros::Time::now();
  while(ros::Time::now()-t_ref<ros::Duration(secs)||secs<0)
  {
    ros::spinOnce();
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  double altitude;
  ROS_INFO("Strategy started");
  //ros::init(argc, argv, "strategy",ros::init_options::NoSigintHandler);
  ros::init(argc, argv, "strategy");
  //signal(SIGINT, basicSigintHandler);
  const int HOVER = 69;
  const int DESTINATION = 42;

  Strategy myStrategy;
  ros::Rate r(20);  // This function refreshes 20 times per second.
  waitSomeSeconds(15.0,r);
  ROS_INFO("taking off");
  myStrategy.setDestination(-1.0, -1.0, -1.0, -1.0);
  myStrategy.publishDestination();
  myStrategy.publishStrategy(HOVER);
  myStrategy.publishTakeoff();
  waitSomeSeconds(5.0,r);
  myStrategy.setDestination(0.0,0.0,0.7,0.0);
  myStrategy.publishDestination();
  myStrategy.publishStrategy(DESTINATION);
  waitSomeSeconds(2.0,r);
  myStrategy.publishMakeKeyframe();
  waitSomeSeconds(1.0,r);
  myStrategy.setDestination(0.0,0.0,1.3,0.0);
  myStrategy.publishDestination();
  while (ros::ok()&&myStrategy.pose.z <  myStrategy.destination.z - 0.05)
  {
    ros::spinOnce();
    r.sleep();
  }
  waitSomeSeconds(2.0,r);
  myStrategy.publishMakeKeyframe();
  waitSomeSeconds(2.0,r);
  myStrategy.setDestination(0.0,0.0,0.7,0.0);
  myStrategy.publishDestination();
  ROS_INFO("entering infinite loop");
  waitSomeSeconds(-1,r);


  myStrategy.publishLand();

  return 0;
}
