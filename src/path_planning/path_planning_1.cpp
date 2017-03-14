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

#include "boris_drone/path_planning_1.h"

PathPlanning::PathPlanning()
{
  // drone prefix from the launch file.
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);

  // List of subscribers and publishers. This node subscribes to pose_estimation to get the
  // real-time
  // position of the drone. It also subsribes to Stragegy in order to know in what the drone must do
  // and so where it has to go.
  // This node publishes in the node path_planning that communicates with the controller and to the
  // node mapcell that is just used to export data from tests.

  // Subscribers
  pose_channel = nh.resolveName("pose_estimation");
  pose_sub = nh.subscribe(pose_channel, 10, &PathPlanning::poseCb, this);

  strategy_channel = nh.resolveName("strategy");
  strategy_sub = nh.subscribe(strategy_channel, 10, &PathPlanning::strategyCb, this);

  // Publishers
  poseref_channel = nh.resolveName("path_planning");
  poseref_pub = nh.advertise< boris_drone::PoseRef >(poseref_channel, 1);

  explore_channel = nh.resolveName("finished_exploring");
  explore_pub = nh.advertise<std_msgs::Empty>(explore_channel, 1);

  // instruction_publishing = false;
}

// Destructor

PathPlanning::~PathPlanning()
{
}

// Function to reset the desired pose sent to the controller
void PathPlanning::reset()
{
  next_x = 0;
  next_y = 0;
  next_z = 1;
  next_rotZ = 0;
  i = 0;
  gridInitialized = false;

  // XMax and YMax are the half of the distance between the horizontal and vertical border lines of
  // the vision of the drone. Those values come from bottom camera data and 0.8 is used as a
  // security factor in the case of the drone is not well oriented.

  XMax = next_z * 0.435 * 0.8 * 0.5;
  YMax = next_z * 0.326 * 0.8 * 0.5;
}

// This function publish the position where the drone has to get. It is used as pose_ref in the
// controller node.
void PathPlanning::publish_poseref(bool takeoff, bool land)
{
  // instantiate the poseref message
  boris_drone::PoseRef poseref_msg;

  poseref_msg.x = next_x;
  poseref_msg.y = next_y;
  poseref_msg.z = next_z;
  poseref_msg.rotZ = next_rotZ;
  poseref_msg.landAndStop = land;
  poseref_msg.takeoffAndStart = takeoff;


  poseref_pub.publish(poseref_msg);
}

// This function is called when this node receives a message from the topic "pose_estimation". So it
// takes this message and put it in a variable where it will be used in the other functions.

void PathPlanning::poseCb(const boris_drone::Pose3D::ConstPtr posePtr)
{
  pose = *posePtr;
}

// This function is called when this node receives a message from the topic "strategy". So it
// takes this message and puts it in a variable where it will be used in the other functions.

void PathPlanning::strategyCb(const std_msgs::Int16::ConstPtr strategyPtr)
{
  strategy = strategyPtr->data;
}

int PathPlanning::getStrategy()
{
  return strategy;
}

// First approach to explore a map. The drones makes a kind of labyrinth

bool PathPlanning::xy_desired()
{
  // The drone is considered as "arrived on poseref if it in a radius of 0.35m"
  if (sqrt((pose.x - next_x) * (pose.x - next_x) +
           (pose.y - next_y) * (pose.y - next_y)) < 0.35)
  {
    printf("i: %d\n", i);
    if (i % 2 == 0)
    {
      next_x += 1;
    }
    else if (i % 4 == 1)
    {
      next_y += 1;
    }
    else  //(i%4==3)
    {
      next_y -= 1;
    }
    i++;
    return true;
  }
  else
  {
    return false;
  }
}

// In our final approach to expore the map, the pathplanning creates a grid. This grid is made of
// 0.10*0.10 m² cells that can be in 3 different states :
// 0 means unseen/unexplored
// 1 means seen/explored
// 2 means border cell explored (there is cells behind but we haven't seen them yet)
// 3 means wall cell detected

void PathPlanning::InitializeGrid()
{
  // Set all the cells to "unexplored"
  grid_origin_x = pose.x - SIDE/2.0;
  grid_origin_y = pose.y - SIDE/2.0;
  for (i = 0; i < SIDE * 10; i++)
  {
    for (j = 0; j < SIDE * 10; j++)
    {
      myGrid[i][j] = 0;
    }
  }
  gridInitialized = true;
  gridSize = SIDE*SIDE*100;
  nExploredCell = 0;
}


// This function trasnforms the XMax and Ymax into the panel of cells that are seen by the drone in
// function of its position.
// !! abscisse = I and ordoonee = J
void PathPlanning::GetFieldOfView(double x, double y, int* iMin, int* iMax, int* jMin, int* jMax)
{

  int* ijMax = XYToCell(x+XMax,y+YMax);
  int* ijMin = XYToCell(x-XMax,y-YMax);
  *iMax = ijMax[0];
  *jMax = ijMax[1];
  *iMin = ijMin[0];
  *jMin = ijMin[1];
}

// This function transforms i and j coordinates to x and y position in the real playground.
void PathPlanning::CellToXY(int i, int j, double* x, double* y)
{
  ROS_INFO("test: start celltoxy: %f, %f, %d, %d",*x,*y,i,j);
  *x = grid_origin_x + (double)(i * (SIDE/10.0));
  *y = grid_origin_y + (double)(j * (SIDE/10.0));
}

int* PathPlanning::XYToCell(double x, double y)
{
  ROS_INFO("start XYToCell: %f, %f",x,y);
  ROS_INFO("grid origin: %f, %f",grid_origin_x,grid_origin_y);
  int i = (int)((x - grid_origin_x) / (SIDE/10.0));
  int j = (int)((y - grid_origin_y) / (SIDE/10.0));
  ROS_INFO("mid XYToCell: %f, %f",x,y);
  int ij[2] = {i, j};
  return ij;
}


void PathPlanning::UpdateMap(double x, double y)
{
  ROS_INFO_THROTTLE(3,"I'm in the updatemap function");
  int iMin;
  int jMin;
  int iMax;
  int jMax;
  GetFieldOfView(x, y, &iMin, &iMax, &jMin, &jMax);
  boris_drone::cellUpdate cellUpdateMsg;
  for (i = iMin; i < iMax; i++)
  {
    for (j = jMin; j < jMax; j++)
    {
      if (myGrid[i][j] != EXPLORED)
      {
        nExploredCell++;
        if (i == iMin || i == iMax - 1 || j == jMin || j == jMax - 1)
        {
          myGrid[i][j] = BORDER;
        }
        else
        {
          myGrid[i][j] = EXPLORED;
        }
      }
    }
  }
  ROS_INFO_THROTTLE(3,"Map updated. Explored %d/%d cells",nExploredCell,gridSize);
}

// This function returns the squared distance between two cells.
double PathPlanning::sqdistance(int i, int j, int k, int l)
{
  return (i - k)*(i - k) + (j - l)*(j - l);
}

// This function replaced the labyrtinth one. It takes as argument the position of the drone and
// gives back the best cell where the drone must go. In order to do that, it compares all the border
// cells of the map and tell the drone to go to the nearrest one.
void PathPlanning::advanced_xy_desired()
{
  ROS_INFO("start advanced_xy_desired");
  double k;
  double l;
  int* ij = XYToCell(pose.x,pose.y);
  int drone_i = ij[0];
  int drone_j = ij[1];
  double bestDist = 1000000.0;
  double currDist;
  int closestJ = 0;
  int closestI = 0;
  ROS_INFO("before for");
  for (i = 0; i < SIDE * 10; i++)
  {
    for (j = 0; j < SIDE * 10; j++)
    {
      currDist = sqdistance(drone_i, drone_j, i, j);
      if (myGrid[i][j] == BORDER &&  currDist < bestDist)
      {
        bestDist = currDist;
        closestJ = j;
        closestI = i;
      }
    }
  }
  ROS_INFO("before if");
  if (bestDist == 1000000.0)
  {
    std_msgs::Empty empty_msg;
    explore_pub.publish(empty_msg);
    ROS_INFO("The map was explored completely, there are no more border cells!");
  }
  ROS_INFO("test: beforecelltoxy");
  CellToXY(closestI, closestJ, &k, &l);
  SetRef(k,l,next_z,0);
}

// This function give to the object variable the computed next reference position.
void PathPlanning::SetRef(double x_ref, double y_ref, double z_ref, double rotZ_ref)
{
  this->next_x = x_ref;
  this->next_y = y_ref;
  this->next_z = z_ref;
  this->next_rotZ = rotZ_ref;
}

// Main function, will publish pose_ref with regard to the selected strategy comming from the
// Strategy node.
int main(int argc, char** argv)
{
  double *poseRefX, *poseRefY;
  ros::init(argc, argv, "path_planning");
  PathPlanning myPath;
  ros::Rate r(20);  // Refresh every 1/20 second.
  int strategy = myPath.getStrategy();
  ROS_DEBUG("path planning initialized");

  myPath.reset();
  myPath.publish_poseref(false,false);
  ros::spinOnce();
  r.sleep();
  ROS_INFO("poseref initialized and launched");
  while (ros::ok())
  {
    TIC(path);
    strategy = myPath.getStrategy();
    if (strategy == WAIT)//0
    {
      myPath.publish_poseref(false,false);
    }
    else if (strategy == TAKEOFF)//1
    {
      myPath.publish_poseref(true,false);
    }
    else if (strategy == SEEK)//2
    {
      myPath.SetRef(myPath.pose.x + 1.0, 0, myPath.next_z, 0);
      myPath.publish_poseref(false,false);
    }
    else if (strategy == EXPLORE)//3
    {
      if (!myPath.gridInitialized)
      {
        ROS_INFO("I am initializing the grid");
        myPath.InitializeGrid();
      }
      myPath.UpdateMap(myPath.pose.x, myPath.pose.y);
      ROS_INFO("before advancexydesired");
      myPath.advanced_xy_desired();
      ROS_INFO("poseref is : (%lf, %lf)", *poseRefX, *poseRefY);

      ros::Duration(1).sleep();
      myPath.publish_poseref(false,false);
    }
    else if (strategy == BACKTOBASE)
    {
      myPath.SetRef(0,0,myPath.next_z,myPath.pose.rotZ);
      myPath.publish_poseref(false,false);
    }
    else if (myPath.getStrategy() == LAND)
    {
      ROS_INFO("landing");
      myPath.SetRef(0,0,0,0);
      myPath.publish_poseref(false,true);
    }


    //TOC(path, "path planning");
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
