/*!
 *  \file opencv_utils.h
 *  \brief opencv_utils contains functions to handle
 *         frame transformation with opencv according
 *         with the Pose3D message definition
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of boris_drone.
 */

#ifndef boris_drone_OPENCV_UTILS_H
#define boris_drone_OPENCV_UTILS_H

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <boris_drone/Pose3D.h>
#include <boris_drone/boris_drone.h>
#include <boris_drone/PointXYZRGBSIFT.h>

//! \return the rotation matrix given the roll angle (around x axis)
cv::Mat rotationMatrixX(const double angle);

//! \return the rotation matrix given the pitch angle (around y axis)
cv::Mat rotationMatrixY(const double angle);

//! \return the rotation matrix given the yaw angle (around z axis)
cv::Mat rotationMatrixZ(const double angle);

//! \return the rotation matrix given the roll,pitch,yaw angles
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw);

//This function gives the same reslt as the transpose of the above function,
//but it is clearer for me to have two different functions
cv::Mat rollPitchYawToChangeBaseMatrix(const double roll, const double pitch, const double yaw);

//! \get rotation matrix used by cv
void getCameraPositionMatrices(const boris_drone::Pose3D& pose, cv::Mat& R, cv::Mat& T, bool front);

//! \return the transformation (rotation and transltation) matrix
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz);

//! Debug for the transformation (rotation and transltation) matrix
void debugRTMatrix(cv::Mat Rt);

//! This method converts opencv keypoint coordinates to opencv point format
std::vector< cv::Point2f > Points(const std::vector< cv::KeyPoint >& keypoints);

double getSqDist(boris_drone::Pose3D pose1, boris_drone::Pose3D pose2);

double angleBetween(const cv::Mat &p1, const cv::Mat &p2);

void getFundamentalMatrix(const cv::Mat &P1, const cv::Mat &P2, cv::Mat F);

#endif /* boris_drone_OPENCV_UTILS_H */
