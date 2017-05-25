
#ifndef boris_drone_MAP_H
#define boris_drone_MAP_H
#define PCL_NO_PRECOMPILE

#define USE_PROFILING
#include <boris_drone/profiling.h>

/* Header files */
#include <boris_drone/boris_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <set>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/contrib/contrib.hpp>

/* Point Cloud library */
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>              // pcl::PointCloud

/* Boost */
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/* Messages */
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>

/* Sparse Bundle Adjustment */
#include <cvsba/cvsba.h>

/* boris_drone */
#include <boris_drone/PointXYZRGBSIFT.h>  // pcl::PointXYZRGBSIFT
#include <boris_drone/Pose3D.h>
#include <boris_drone/ProcessedImageMsg.h>
#include <boris_drone/BundleMsg.h>
#include <boris_drone/TargetDetected.h>
#include <boris_drone/map/projection_2D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/read_from_launch.h>

#include <boris_drone/map/keyframe.h>
#include <boris_drone/map/frame.h>
#include <boris_drone/map/map_utils.h>

/*!
 * \class Map
 * This object wraps functions to execute the mapping node
 */
class Map
{
private:
  ros::Publisher bundle_pub;
  std::string    bundle_channel;

  /* Tresholds for PnP */
  int threshold_lost;                        //!< min nber of matching keypoints for visual pose
  int threshold_new_keyframe;                //!< below this nber of matching keypoints => new keyframe
  double threshold_new_keyframe_percentage;  //!< below this pct of matching keypoints => new keyframe

  /* bools */
  bool tracking_lost;  //!< true if the last visual information does not permit to estimate the drone pose
  bool do_search;      //!< parameter: if true the previous keyframe are used to perform searches
  bool stop_if_lost;   //!< parameter: if true the mapping stops when tracking_lost is true

  Keyframe* reference_keyframe;  //!< The reference Keyframe is the last matching keyframe

  ros::NodeHandle* nh;
  void resetList();

  /*!
   * This method computes the PnP estimation
   * @param[in]  current_frame The frame containing keypoints of the last camera observation
   * @param[out] PnP_pose      The visual pose estimation
   * @param[out] inliers       The indexes of keypoints with a correct matching
   * @param[in]  ref_keyframe  The Keyframe used to perform the estimation
   */
  int doPnP(const Frame& current_frame, boris_drone::Pose3D& PnP_pose);

  cv::Mat camera_matrix_K;

  cv::Mat tvec;  //!< last translation vector (PnP estimation)
  cv::Mat rvec;  //!< last rotational vector (PnP estimation)


  cv::Mat cam_plane_top;
  cv::Mat cam_plane_bottom;
  cv::Mat cam_plane_left;
  cv::Mat cam_plane_right;

  //! This method initializes planes defining the visible area from the camera (according to camera parameters)
  void initPlanes();

  /*!
   * This method determines if a new Keyframe is needed
   * @param[in] number_of_common_keypoints Number of common keypoints between reference Keyframe and last Frame
   * @return true if a  new Keyframe is needed, false otherwise
   */
  bool keyframeNeeded(boris_drone::Pose3D pose);

  /*!
   * This method creates a new keyframe, sets it as the reference keyframe, and pushes it on the keyframe list
   */
  void newReferenceKeyframe(const Frame& current_frame, boris_drone::Pose3D PnP_pose, bool PnP_success);

public:
  //! Contructor. Initialize an empty map
  Map();
  Map(ros::NodeHandle* nh, bool do_search, bool stop_if_lost, cv::Mat camera_matrix_K);

  //! Destructor.
  ~Map();

  cv::FlannBasedMatcher matcher;  //!< object for descriptors comparison
  std::vector<Keyframe*> keyframes;

  //! The cloud object containing 3D points
  pcl::PointCloud<pcl::PointXYZRGBSIFT>::Ptr cloud;

  cv::Mat descriptors; //!< descriptors in opencv format

  bool processFrame(const Frame& frame,boris_drone::Pose3D& PnP_pose);

  void newKeyframe(const Frame& frame);

  void resetPose();

  void doBundleAdjustment(std::vector<Keyframe*> kfs, bool fixed_poses);

  void getVisualPose();

  int matchWithFrame(const Frame& frame, std::vector<cv::Point3f>& inliers_map_matching_points,
                                          std::vector<cv::Point2f>& inliers_frame_matching_points);

  void targetDetectedPublisher();

  void updateBundle(const boris_drone::BundleMsg::ConstPtr bundlePtr);
};

#endif /* boris_drone_MAP_H */
