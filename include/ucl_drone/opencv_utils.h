/*!
 *  \file opencv_utils.h
 *  \brief opencv_utils contains functions to handle
 *         frame transformation with opencv according
 *         with the Pose3D message definition
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone.
 */

#ifndef ucl_drone_OPENCV_UTILS_H
#define ucl_drone_OPENCV_UTILS_H

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ucl_drone.h>

//! \return the rotation matrix given the roll angle (around x axis)
cv::Mat rotationMatrixX(const double angle);

//! \return the rotation matrix given the pitch angle (around y axis)
cv::Mat rotationMatrixY(const double angle);

//! \return the rotation matrix given the yaw angle (around z axis)
cv::Mat rotationMatrixZ(const double angle);

/** Get rotation matrix from RPY angles
 * If yaw angle is turned around z axis, then pitch angle around y axis, and finally roll angle around x axis,
 * the result is a rotation matrix whose columns are the coordinates of the new base expressed in the old base.
 * Example:
 * pt_world = 3D vector in world coordinates
 * A = rollPitchYawToRotationMatrix(roll,pitch,yaw)
 * pt_cam = A.t()*pt_world;
 * Where roll, pitch and yaw are the angles to rotate to obtain a coordinate system where
 * z points forward, x points right, y points down from the point of view of the camera
 * \return the rotation matrix given the roll,pitch,yaw angles
 */
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw);

//! \get rotation matrix used by cv
void getCameraPositionMatrices(const ucl_drone::Pose3D& pose, cv::Mat& R, cv::Mat& T, bool front);

//! \return the transformation (rotation and transltation) matrix
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz);

//! Debug for the transformation (rotation and transltation) matrix
void debugRTMatrix(cv::Mat Rt);

//! This method converts opencv keypoint coordinates to opencv point format
std::vector<cv::Point2f> Points(const std::vector< cv::KeyPoint >& keypoints);

#endif /* ucl_drone_OPENCV_UTILS_H */
