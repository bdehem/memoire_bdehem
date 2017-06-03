#ifndef boris_drone_LANDMARK_H
#define boris_drone_LANDMARK_H

#include <opencv2/core/core.hpp>
#include <set>
#include <ros/ros.h>


struct Landmark
{
private:
  static int ID_counter;
public:
  std::set<int> keyframes_seeing;
  cv::Point3d coordinates;
  cv::Mat descriptor;
  int ID;
  Landmark(cv::Point3d& coordinates, cv::Mat& descriptor);
  Landmark();

  //! Destructor.
  ~Landmark();

  void updateCoords(cv::Point3d& coordinates);

  void setAsSeenBy(int kfID);
  bool setAsUnseenBy(int kfID);
  void print();
};

#endif /* boris_drone_LANDMARK_H */
