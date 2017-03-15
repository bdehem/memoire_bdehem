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

}

// Destructor

PathPlanning::~PathPlanning()
{
}

// Function to reset the desired pose sent to the controller
void PathPlanning::reset()
{
  height_of_flight = 1.5;
  gridInitialized = false;

  // XMax and YMax are the half of the distance between the horizontal and vertical border lines of
  // the vision of the drone. Those values come from bottom camera data and 0.8 is used as a
  // security factor in the case of the drone is not well oriented.
  XMax = height_of_flight * 0.435 * 0.8 * 0.5;
  YMax = height_of_flight * 0.326 * 0.8 * 0.5;
}

// This function publish the position where the drone has to get. It is used as pose_ref in the
// controller node.
void PathPlanning::publish_poseref(double x_ref, double y_ref, double z_ref, double rotZ_ref, bool takeoff, bool land)
{
  // instantiate the poseref message
  boris_drone::PoseRef poseref_msg;

  poseref_msg.x = x_ref;
  poseref_msg.y = y_ref;
  poseref_msg.z = z_ref;
  poseref_msg.rotZ = rotZ_ref;
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
  for (int i = 0; i < SIDE * 10; i++)
  {
    for (int j = 0; j < SIDE * 10; j++)
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
  XYToCell(x+XMax,y+YMax,iMax,jMax);
  XYToCell(x-XMax,y-YMax,iMin,jMin);
  int idrone;
  int jdrone;
  XYToCell(x,y,&idrone,&jdrone);
//  std::string str = "\n";
//  for (int i = -10; i< 20;i++)
//  {
//    for (int j = -10 ; j<20 ; j++)
//    {
//      if (i==idrone    && j==jdrone)
//      {
//        str.append("D");
//      }
//      else if (*iMin <= i && i <= *iMax && *jMin <= j && j <= *jMax)
//      {
//        str.append("+");
//      }
//      else if (i<0 || i>9 || j<0 || j > 9)
//      {
//        str.append(" ");
//      }
//      else
//      {
//        str.append(".");
//      }
//    }
//    str.append("\n");
//  }
//  std::cout << str << '\n';
}

// This function transforms i and j coordinates to x and y position in the real playground.
void PathPlanning::CellToXY(int i, int j, double* x, double* y)
{
  *x = grid_origin_x + (double)(i * (SIDE/10.0));
  *y = grid_origin_y + (double)(j * (SIDE/10.0));
}

void PathPlanning::XYToCell(double x, double y, int* i, int* j)
{
  *i = ((x - grid_origin_x) / (SIDE/10.0));
  *j = ((y - grid_origin_y) / (SIDE/10.0));
}

//void PathPlanning::XYToCell(double x, double y, int* i, int* j)
//{
//  ROS_INFO("start XYToCell:x,y = %f, %f",x,y);
//  double xrel = x - grid_origin_x;
//  double yrel = y - grid_origin_y;
//  ROS_INFO("xrel,yrel = %f, %f",xrel,yrel);
//  double di = xrel / (SIDE/10.0);
//  double dj = yrel / (SIDE/10.0);
//  ROS_INFO("di,dj = %f, %f",di,dj);
//  int ii = di;
//  int jj = dj;
//  ROS_INFO("ii,jj = %d, %d",ii,jj);
//  *i = ii;
//  *j = jj;
//  ROS_INFO("end XYToCell:i,j = %d, %d",*i,*j);
//}

int PathPlanning::inGrid(int i)
{
  return std::max(0,std::min(10*SIDE-1,i));
}

void PathPlanning::UpdateMap(double x, double y)
{
  if (!gridInitialized)
  {
    InitializeGrid();
  }
  int iMin;
  int jMin;
  int iMax;
  int jMax;
  GetFieldOfView(x, y, &iMin, &iMax, &jMin, &jMax);

  for (int i = inGrid(iMin); i <= inGrid(iMax); i++)
  {
    for (int j = inGrid(jMin); j <= inGrid(jMax); j++)
    {
      if (myGrid[i][j] != EXPLORED)
      {
        if (i == iMin || i == iMax || j == jMin || j == jMax )
        {
          myGrid[i][j] = BORDER;
        }
        else
        {
          myGrid[i][j] = EXPLORED;
          nExploredCell++;
        }
      }
    }
  }
//  ROS_INFO_THROTTLE(3,"Map updated. Explored %d/%d cells",nExploredCell,gridSize);
//  ROS_INFO_THROTTLE(3,"current FOV: i=%d-%d; j=%d-%d",iMin,iMax,jMin,jMax);
//  ROS_INFO_THROTTLE(3,"=====================================Map Updated==============================================");

  if (nExploredCell == gridSize)
  {
    std_msgs::Empty empty_msg;
    explore_pub.publish(empty_msg);
    ROS_INFO("The map was explored completely, there are no more border cells!");
  }
}

// This function returns the squared distance between two cells.
int PathPlanning::sqdistance(int i, int j, int k, int l)
{
  return (i - k)*(i - k) + (j - l)*(j - l);
}

// This function replaced the labyrtinth one. It takes as argument the position of the drone and
// gives back the best cell where the drone must go. In order to do that, it compares all the border
// cells of the map and tell the drone to go to the nearrest one.
void PathPlanning::advanced_xy_desired(double* x, double* y)
{
  int i_pos;
  int j_pos;
  XYToCell(pose.x,pose.y,&i_pos,&j_pos);
  int bestDist = -1;
  int currDist;
  int closestJ;
  int closestI;

  for (int i = 0; i < SIDE * 10; i++)
  {
    for (int j = 0; j < SIDE * 10; j++)
    {
      currDist = sqdistance(i, j, i_pos, j_pos);
      if (myGrid[i][j]==BORDER  &&  (currDist<bestDist || bestDist==-1) )
      {
        bestDist = currDist;
        closestJ = j;
        closestI = i;
      }
    }
  }
  if (bestDist == -1)
  {
    *x = pose.x;
    *y = pose.y;
  }
  else
  {
    CellToXY(closestI, closestJ, x, y);
  }
  std::string str = "\n";
  for (int i = -10; i< 20;i++)
  {
    for (int j = -10 ; j<20 ; j++)
    {
      if (i==i_pos    && j==j_pos)
      {
        str.append("D");
      }
      else if (i==closestI && j==closestJ)
      {
        str.append("+");
      }
      else if (i<0 || i>9 || j<0 || j > 9)
      {
        str.append(" ");
      }
      else
      {
        switch(myGrid[i][j])
        {
          case EXPLORED:
            str.append("X");
            break;
          case UNEXPLORED:
            str.append(".");
            break;
          case BORDER:
            str.append("_");
            break;
        }
      }
    }
    str.append("\n");
  }
  std::cout << str << '\n';
  printf("Explored %d/100 cells\n",nExploredCell);
  printf("Drone  : %d,%d  --- %f,%f\n",i_pos,j_pos,pose.x,pose.y);
  printf("Target : %d,%d\n",closestI,closestJ);
}

void PathPlanning::wait()
{
  publish_poseref(0,0,height_of_flight,0,false,false);
}

void PathPlanning::takeOff()
{
  publish_poseref(0,0,height_of_flight,0,true,false);
}

void PathPlanning::seek()
{
  publish_poseref(pose.x+1.0, 0,height_of_flight,0,false,false);
}

void PathPlanning::explore()
{
  UpdateMap(pose.x, pose.y);
  double* x;
  double* y;
  advanced_xy_desired(x,y);
  publish_poseref(*x,*y,height_of_flight,0,false,false);
}

void PathPlanning::backToBase()
{
  publish_poseref(0,0,height_of_flight,0,false,false);
}

void PathPlanning::land()
{
    publish_poseref(0,0,0,0,false,true);
    ROS_INFO("landing");
}

// Main function, will publish pose_ref with regard to the selected strategy coming from the
// Strategy node.
int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  PathPlanning myPath;
  myPath.reset();
  ros::Rate r(20);  // Refresh every 1/20 second.
  myPath.wait();
  ros::spinOnce();
  r.sleep();
  ROS_INFO("poseref initialized and launched");
  while (ros::ok())
  {
    TIC(path);
    switch(myPath.getStrategy())
    {
      case WAIT:
        myPath.wait();
        break;
      case TAKEOFF:
        myPath.takeOff();
        break;
      case SEEK:
        myPath.explore();
//        myPath.seek();
        break;
      case EXPLORE:
        myPath.explore();
        break;
      case BACKTOBASE:
        myPath.backToBase();
        break;
      case LAND:
        myPath.land();
        break;
    }

    //TOC(path, "path planning");
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
