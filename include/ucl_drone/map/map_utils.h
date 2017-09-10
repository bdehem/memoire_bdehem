/*!
 *  \file map_utils.h
 *  \brief This header file contains various utility functions for the mapping task
 *  \authors Boris Dehem
 *  \year 2017
 */

#ifndef ucl_drone_MAPUTILS_H
#define ucl_drone_MAPUTILS_H

#include <ucl_drone/profiling.h>
#include <ucl_drone/ucl_drone.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/opencv_utils.h>
// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/shared_ptr.hpp>

#include <ucl_drone/map/camera.h>
#include <ucl_drone/map/keyframe.h>


/**
 * Obtain matches between two sets of descriptors
 * @param[in]  descriptors1       First set of descriptors
 * @param[in]  descriptors2       Second set of descriptors
 * @param[out] matching_indices_1 Indices of matches in the first set
 * @param[out] matching_indices_2 Indices of matches in the second set
 * @param[in]  threshold          Distance threshold for a pair to be considered a match
 * @param[in]  max_matches        Maximal number of matches allowed (no limit if maxmatches is negative)
 */
void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold, int max_matches);

/**
 * Obtain matches between two sets of descriptors
 * Used when both sets come from keyframes (not from the map).
 * No match is considered if either observation already corresponds to a mapped landmark,
 * in that case it must be matched with the map
 * @param[in]  descriptors1       First set of descriptors
 * @param[in]  descriptors2       Second set of descriptors
 * @param[in]  ptIDs1             Point IDs of first set in the first keyframe (negative for unmatched points)
 * @param[in]  ptIDs2             Point IDs of first set in the second keyframe (negative for unmatched points)
 * @param[out] matching_indices_1 Indices of matches in the first set
 * @param[out] matching_indices_2 Indices of matches in the second set
 * @param[in]  threshold          Distance threshold for a pair to be considered a match
 * @param[in]  max_matches        Maximal number of matches allowed (no limit if maxmatches is negative)
 */
void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  const std::vector<int>& ptIDs1, const std::vector<int>& ptIDs2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold, int max_matches);

/**
 * Obtain Distance between two poses (disregarding rotations)
 * @param[in] pose0 First pose
 * @param[in] pose1 Second pose
 * @return Distance between the two keyframes
 */
double poseDistance(const ucl_drone::Pose3D& pose0, const ucl_drone::Pose3D& pose1);

/**
 * Obtain the 3D position of a point from two observations from known locations
 * @param[out] pt_out 3D position of the point
 * @param[in]  kf1    First keyframe observing the point
 * @param[in]  kf2    Second keyframe observing the point
 * @param[in]  idx1   Index of the point in the first keyframe
 * @param[in]  idx2   Index of the point in the second keyframe
 */
bool triangulate(cv::Point3d& pt_out, Keyframe *kf1, Keyframe *kf2, int idx1, int idx2);

/**
 * Check whether a is within the field of view of a kayframe
 * @param[in] kf      Keyframe to check
 * @param[in] point3D coordinated of point to test
 * @param[in] thresh  Threshold. 0 to strictly check whether point is visible, negative to be more tolerant, positive to be more strict
 * @return true if the point is withun the keyframes field of view
 */
bool pointIsVisible(const Keyframe kf, const cv::Point3d& point3D, double thresh);

/**
 * Inner function used by triangulate
 */
inline void getVandEpsil(cv::Mat& V, cv::Mat& epsil, cv::Mat& x_hat, cv::Mat& x1_hat,
                      cv::Mat& x_tilde, cv::Mat& x1_tilde, double f);

/**
* Inner function used by triangulate
*/
void getFundamentalMatrix(const cv::Mat &P1, const cv::Mat &P2, cv::Mat F);

#endif /*ucl_drone_MAPUTILS_H*/
