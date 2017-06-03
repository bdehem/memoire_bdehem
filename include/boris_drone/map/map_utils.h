
#ifndef boris_drone_MAPUTILS_H
#define boris_drone_MAPUTILS_H


#include <boris_drone/boris_drone.h>
#include <boris_drone/Pose3D.h>
#include <boris_drone/opencv_utils.h>
// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/shared_ptr.hpp>

#include <boris_drone/map/camera.h>
#include <boris_drone/map/keyframe.h>



void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold, int max_matches);

bool triangulate(cv::Point3d& pt_out, Keyframe *kf1, Keyframe *kf2, int idx1, int idx2);

inline void getVandEpsil(cv::Mat& V, cv::Mat& epsil, cv::Mat& x_hat, cv::Mat& x1_hat,
                      cv::Mat& x_tilde, cv::Mat& x1_tilde, double f);

#endif /*boris_drone_MAPUTILS_H*/
