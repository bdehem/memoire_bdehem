/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/mapping_node.h>

Frame::Frame()
{
}

Frame::Frame(ucl_drone::ProcessedImageMsg::ConstPtr msg)
{
  this->pose = msg->pose;

  // convert msg to opencv format
  this->img_points.resize(msg->keypoints.size());
  this->descriptors = cv::Mat_<float>(msg->keypoints.size(), DESCRIPTOR_SIZE);
  this->image = msg->image;

  for (unsigned i = 0; i < msg->keypoints.size(); ++i)
  {
    this->img_points[i].x = (double)msg->keypoints[i].point.x;
    this->img_points[i].y = (double)msg->keypoints[i].point.y;
    for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
    {
      this->descriptors.at<float>(i, j) = (float)msg->keypoints[i].descriptor[j];
    }
  }
}

Frame::~Frame()
{
}
