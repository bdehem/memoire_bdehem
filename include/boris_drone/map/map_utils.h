
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


void matchDescriptors(const cv::Mat& descriptors1, const cv::Mat& descriptors2,
  std::vector<int>& matching_indices_1, std::vector<int>& matching_indices_2, double threshold);

bool triangulate(cv::Point3d& pt_out, const cv::Point2d& pt1, const cv::Point2d& pt2,
            const boris_drone::Pose3D& pose1, const boris_drone::Pose3D& pose2,
            Camera * cam1, Camera * cam2);

inline void getVandEpsil(cv::Mat& V, cv::Mat& epsil, cv::Mat& x_hat, cv::Mat& x1_hat,
                      cv::Mat& x_tilde, cv::Mat& x1_tilde, double f);

#endif /*boris_drone_MAPUTILS_H*/
