/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/mapping_node.h>

Frame::Frame()
{
}

Frame::Frame(boris_drone::ProcessedImageMsg::ConstPtr msg)
{
  this->pose = msg->pose;

  // convert msg to opencv format
  this->imgPoints.resize(msg->keypoints.size());
  this->descriptors = cv::Mat_<float>(msg->keypoints.size(), DESCRIPTOR_SIZE);

  for (unsigned i = 0; i < msg->keypoints.size(); ++i)
  {
    this->imgPoints[i].x = (double)msg->keypoints[i].point.x;
    this->imgPoints[i].y = (double)msg->keypoints[i].point.y;
    for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
    {
      this->descriptors.at<float>(i, j) = (float)msg->keypoints[i].descriptor[j];
    }
  }
}

Frame::~Frame()
{
}
