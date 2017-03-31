/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <boris_drone/map/map_keyframe_based.h>

Frame::Frame()
{
}

Frame::Frame(boris_drone::ProcessedImageMsg::ConstPtr msg)
{
  this->msg = *msg;

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

/*!
 * @param[in]  idx_points  index of points in this->imgPoints to convert
 * @param[in]  points_out  ???
 * @param[in]  keyframe_ID ???
 * @param[out] pointcloud  points converted to PointXYZRGBSIFT
 */
void Frame::convertToPcl(std::vector<int>& idx_points, std::vector<cv::Point3f> points_out,
                         int keyframe_ID, pcl::PointCloud<pcl::PointXYZRGBSIFT>::Ptr& pointcloud)
{
  pointcloud->points.resize(idx_points.size());  // prepares output structure
  // for each points selected by idx_points
  for (unsigned i = 0; i < idx_points.size(); i++)
  {
    int j = idx_points[i];

    pcl::PointXYZRGBSIFT point;  // initialize a new PCL point

    // Fill coordinates
    point.x = points_out[i].x;
    point.y = points_out[i].y;
    point.z = points_out[i].z;

    // Other fields
    point.view_count = 1;
    point.keyframe_ID = keyframe_ID;

    int rgb_ = 255 << 16 | 0 << 8 | 0;  // red by default
    point.rgb = *reinterpret_cast< float* >(&rgb_);

    // Fill the corresponding description
    for (int k = 0; k < DESCRIPTOR_SIZE; k++)
    {
      point.descriptor[k] = this->descriptors.at< float >(j, k);
    }

    // add the PCL point to the output
    pointcloud->points[i] = point;
  }
}

void Frame::convertToPcl(                              std::vector<cv::Point3f> points_out,
                         int keyframe_ID, pcl::PointCloud<pcl::PointXYZRGBSIFT>::Ptr& pointcloud)
{
  std::vector<int> idx_points;
  idx_points.resize(this->imgPoints.size());
  for (unsigned k = 0; k < this->imgPoints.size(); k++)
  {
    idx_points[k] = k;
  }
  convertToPcl(idx_points, points_out, keyframe_ID, pointcloud);
}
