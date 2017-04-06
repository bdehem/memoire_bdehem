#ifndef boris_drone_KEYPOINT_H
#define boris_drone_KEYPOINT_H

struct Observation
{
  cv::Point2f point_location;     //Location of the point on the image
  boris_drone::Pose3D drone_pose; //Drone pose at moment of image capture
  Observation(cv::Point2f loc, boris_drone::Pose3D pose) : point_location(loc),drone_pose(pose)
  {
  }
};

struct Keypoint
{
  std::vector<Observation> observations;
  int index; //index in the cloud
};

struct AllKeypoints
{
  std::vector<Keypoint> keypoints;
  cv::Mat descriptors;
};

struct UnmatchedObservations
{
  std::vector<Observation> observations;
  cv::Mat descriptors;
};

#endif
