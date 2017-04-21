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
  // drone prefix and name from the launch file.
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);

  ros::param::get("~drone_name", drone_name);

  // List of subscribers and publishers. This node subscribes to target detection from drone 4,
  // drone 5 and from the drone to which it belongs. This allows this node to always know wich drone
  // sees the target. It is also subscribed to the Multi_drone in order to know which drone is the
  // master and which one is the slave. This gives information of what strategy to chose. The
  // subscription to the master drone navdata gives the information about its battery. This is the
  // only way for the secondary drone to know when it has to start. The subscription to the pose
  // estimation to the secondary drone tells to the main drone when it may go back to the base
  // because the secondary drone is close enough to replace it.
  //
  // This node only publish in the topic strategy that is read from the path_planning.

  // Subscribers
  target_channel = nh.resolveName("boris_drone/target_detected");
  target_sub = nh.subscribe(target_channel, 10, &Strategy::targetDetectedCb, this);

  pose_channel = nh.resolveName("pose_estimation");
  pose_sub = nh.subscribe(pose_channel, 10, &Strategy::poseCb, this);

  explore_channel = nh.resolveName("finished_exploring");
  explore_sub = nh.subscribe(explore_channel,10,&Strategy::exploreCb,this);

  /* ** Other group's comments: ** */
  // TODO: subscribe to channels in funtion of the role ...
  // TODO: do not use absolute path to get the target_detected channel of the current drone
  // TODO: get the other drone name from the DroneRoles message sent by swarm_initialization

  // Publishers
  strategy_channel = nh.resolveName("strategy");
  strategy_pub = nh.advertise<boris_drone::StrategyMsg>(strategy_channel, 1);

  /* ** Other group's comments: ** */
  // TODO: Publisher to send a "ready state" message with the drone_name
  // to the swarm_initialization (multistrategy)
  // the topic name is "ready"
  // the message type is boris_drone::DroneRole
  // fill the drone_name only

  // Initialization of some parameters.

  TargetFound = false;
  FinishedExploring = false;
  go_high = false;
  strategy = WAIT;
}

// Destructor
Strategy::~Strategy()
{
}


// This function gives the position chosen to the object of this function.
void Strategy::SetXYZChosen(float xchosen, float ychosen, float zchosen)
{
  this->xchosen = xchosen;
  this->ychosen = ychosen;
  this->zchosen = zchosen;
}

void Strategy::SetStrategy(int strategy)
{
  this->strategy = strategy;
  this->publish_strategy();
  ROS_INFO("strategy set to %d",strategy);
}

int Strategy::GetStrategy()
{
  return strategy;
}

double Strategy::getAltitude()
{
  return pose.z;
}

// This function sends the strategy number and the position chosen to the path_planning.
void Strategy::publish_strategy()
{
  // instantiate the strategy message
  boris_drone::StrategyMsg strategy_msg;

  strategy_msg.type = strategy;
  if (strategy == GOTO)
  {
    strategy_msg.x = xchosen;
    strategy_msg.y = ychosen;
    strategy_msg.z = zchosen;
  }

  // publish
  strategy_pub.publish(strategy_msg);
}

// This function is called when the topic of the target_detected of this drone publishes something.
void Strategy::targetDetectedCb(const boris_drone::TargetDetected::ConstPtr targetDetectedPtr)
{
  if (TargetFound == 0)
  {
    ROS_INFO("Target Detected!");
    targetX = targetDetectedPtr->world_point.x;
    targetY = targetDetectedPtr->world_point.y;
    TargetFound = 1;
  }
}

void Strategy::poseCb(const boris_drone::Pose3D::ConstPtr posePtr)
{
  pose = *posePtr;
}

void Strategy::exploreCb(const std_msgs::Empty::ConstPtr emptyPtr)
{
  FinishedExploring = true;
}

void Strategy::goHighCb(const std_msgs::Float32::ConstPtr goHighPtr)
{
  go_high = true;
  go_high_altitude = goHighPtr->data;
}

bool Strategy::goingHigh()
{
  return go_high;
}

float Strategy::getHighAltitude()
{
  return go_high_altitude;
}

void Strategy::stopGoingHigh()
{
  go_high = false;
}

// This is the main function where the strategy to sent to the path_planning will be chosen as a
// function of all above data.
int main(int argc, char** argv)
{
  double altitude;
  ROS_INFO("Strategy started");
  ros::init(argc, argv, "strategy");

  Strategy myStrategy;
  ros::Rate r(20);  // This function refreshes every 1/20 second.
  myStrategy.SetStrategy(WAIT);
  ros::spinOnce();
  ros::Duration(10).sleep();
  r.sleep();

  myStrategy.SetStrategy(TAKEOFF);
  int strategy = myStrategy.GetStrategy();
  ros::spinOnce();
  r.sleep();

while (ros::ok())
{
  altitude = myStrategy.getAltitude();
  strategy = myStrategy.GetStrategy();
  if (myStrategy.goingHigh())
  {
    myStrategy.SetXYZChosen(myStrategy.pose.x, myStrategy.pose.y, myStrategy.getHighAltitude());
    myStrategy.SetStrategy(GOTO);
    ros::Duration(10).sleep();
    myStrategy.stopGoingHigh();
  }
  else if (strategy == TAKEOFF && altitude > 0.5)
  {
    ros::Duration(10).sleep();  // Wait for 10s
    myStrategy.SetStrategy(SEEK);         // Change strategy to seek
  }
  else if (strategy == SEEK && myStrategy.TargetFound)
  {
    myStrategy.SetStrategy(EXPLORE);
  }
  else if (strategy == EXPLORE && myStrategy.FinishedExploring)
  {
    myStrategy.SetStrategy(BACKTOBASE);
  }
  else if (strategy == BACKTOBASE &&
          (myStrategy.pose.x * myStrategy.pose.x + myStrategy.pose.y * myStrategy.pose.y < 0.35))
  {
    myStrategy.SetStrategy(LAND);
  }
  ros::spinOnce();
  r.sleep();
}

  return 0;
}
