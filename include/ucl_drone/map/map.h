/*!
 *  \file map.h
 *  \brief This header file contains the map and most mapping functions, as well as the visual pose estimation (PnP)
 *  \authors Boris Dehem
 *  \year 2017
 */

#ifndef ucl_drone_MAP_H
#define ucl_drone_MAP_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/profiling.h>

/* Header files */
#include <ucl_drone/ucl_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <set>
#include <map> //std::map key-value pair

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
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>

/* ucl_drone */
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/BenchmarkInfoMsg.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/BundleMsg.h>
#include <ucl_drone/TargetDetected.h>
#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/read_from_launch.h>

#include <ucl_drone/map/keyframe.h>
#include <ucl_drone/map/frame.h>
#include <ucl_drone/map/landmark.h>
#include <ucl_drone/map/map_utils.h>
#include <ucl_drone/map/camera.h>

/*!
 * \class Map
 * This object wraps functions to execute the mapping task
 */
class Map
{
private:
  // Some static const parameters
  static const int threshold_lost = 10; //!< Drone is considered as lost when it has less RANSAC inliers than this

  //Nodehandle and publisher to communicate with bundle adjuster
  ros::NodeHandle* nh;  //!< pointer to ROS node handle of MappingNode
  std::string    bundle_channel;     //!< Channel for bundles to be adjusted
  ros::Publisher bundle_pub;         //!< Publisher for bundles to be adjusted
  std::string    benchmark_channel;  //!< Channel for benchamrk information
  ros::Publisher benchmark_pub;      //!< Publisher of benchamrk information

  //ROS parameters (can be set in lauch files)
  double thresh_descriptor_match; //!< Threshold for matches between descriptors
  int    max_matches;             //!< Max number of matches when matching sets of descriptors
  bool   no_bundle_adjustment;    //!< If true, bundle adjustment is never performed
  bool   only_init;               //!< If true, no keyframes are created after initialization
  double outlier_threshold; //!< Threshold on contribution to bundle adjustment objective for a point to be considered an outlier
  bool   manual_keyframes;  //!< If true, keyframe decision is not made automatically, but when the user sends a message
  bool   sonar_unavailable; //!< Set true for tests with the drone landed and sonar data is unavailable to use visual data instead
  int    n_kf_local_ba;     //!< Number of keyframes to adjust when running local bundle adjustment
  int    freq_global_ba;    //!< Frequency at which to run global bundle adjustment

  //ROS parameters (used for keyframe needed decision)
  double min_dist; //!< Minimal distance to last keyframe to create a new one
  double min_time; //!< Minimal time between keyframes
  double inliers_thresh; //!< Create a keyframe when less than this amount of RANSAC inliers
  double FOV_thresh;     //!< Create a keyframe when more than this fraction of the screen has no keypoints (from any side)
  double time_thresh;    //!< Create a keyframe when more time than this has elapsed since last one
  double dist_thresh;    //!< Create a keyframe when farther than this from last keyframe

  bool is_adjusting_bundle; //!< True while bundle adjustment is running
  int n_inliers_moving_avg; //!< Average number of inliers in recent frames (far away frames have a lower weight in the average)
  ros::Time last_new_keyframe; //!< Time when a keyframe was last added
  int kf_since_last_global_BA; //!< Number of keyframes created since last time global bundle adjustment was run

  std::vector<double> BA_times;    //!< Times taken by bundle adjustment
  std::vector<int>    BA_num_iter; //!< Number of iterations of bundle adjustment

  Camera camera; //!< Camera used during mapping
  ucl_drone::Pose3D manual_pose; //!< Manual pose (received from user) if using it
  bool                manual_pose_available; //!< true when a manual pose is available

  std::vector<int> landmark_IDs;     //!< Vector of IDs for each landmark in the map
  std::map<int,Landmark*> landmarks; //!< Map of landmark IDs to landmarks
  std::map<int,Keyframe*> keyframes; //!< Map of keyframe IDs to keyframes
  std::map<int,Keyframe*>::iterator first_kf_to_adjust; //!< Iterator to oldest keyframe to include in local bundle adjsutment
  cv::Mat descriptors; //!< descriptors of landmarks

 /**
  * This method computes the PnP estimation
  * @param[in]  current_frame    The frame containing keypoints of the last camera observation
  * @param[out] PnP_pose         The visual pose estimation
  * @param[out] inliers          The indices of keypoints with a correct matching
  * @param[in]  inlier_coverage  fraction of the screen without RANSAC inliers
  */
  int doPnP(const Frame& current_frame, ucl_drone::Pose3D& PnP_pose, int& n_inliers, double& inlier_coverage);

  cv::Mat tvec;  //!< last translation vector (PnP estimation)
  cv::Mat rvec;  //!< last rotational vector (PnP estimation)

 /**
  * Match a frame with the map
  * @param[in]  frame                         The frame to match
  * @param[out] inliers_map_matching_points   The 3D points from the map with a match in the frame
  * @param[out] inliers_frame_matching_points The 2D points from the frame with a match in the map
  * @param[in]  inlier_coverage               fraction of the screen (in the frame) without RANSAC inliers
  */
  int matchWithFrame(const Frame& frame, std::vector<cv::Point3f>& inliers_map_matching_points,
    std::vector<cv::Point2f>& inliers_frame_matching_points, double& inlier_coverage);

 /**
  * Decide whether a new keyframe is needed
  * @param[in]  manual_pose_received True is a manual pose was received (used when manual_keyframes=true)
  * @param[out] n_inliers            Number of RANSAC inliers
  * @param[out] inlier_coverage      Fraction of the screen (in the frame) without RANSAC inliers
  * @param[out] current_pose         Current estimated pose
  * @return true if a new keyframe is needed
  */
  bool keyframeNeeded(bool manual_pose_received, int n_inliers, double inlier_coverage, ucl_drone::Pose3D& current_pose);

 /**
  * Make a new keyframe from a frame
  */
  void newKeyframe(Frame& frame);

 /**
  * Match two keyframes and add any new matches to the map
  * @param[in] kf0 First keyframe to match
  * @param[in] kf1 Second keyframe to match
  */
  void matchKeyframes(Keyframe* kf0, Keyframe* kf1);

 /**
  * Match a keyframe with the map and update the keyframe to reflect new observations
  */
  void matchKeyframeWithMap(Keyframe* kf);

 /**
  * Get points to adjust for bundle adjustment
  * @param[in]  kfIDs  IDs of keyframes to adjust
  * @param[out] points Map of all points seen by at least two of the keyframes in kfIDs
  */
  int getPointsForBA(std::vector<int> &kfIDs, std::map<int,std::map<int,int> > &points);

 /**
  * Prepare a bundle message and send it (to the bundle adjustment node)
  * @param[in] kfIDs     IDs of keyframes to adjust
  * @param[in] is_global If true, disregard kfIDs, and use all keyframes
  */
  void doBundleAdjustment(std::vector<int> kfIDs, bool is_global);
  void targetDetectedPublisher();

 /**
  * Add a landmark to the map
  */
  int addPoint(cv::Point3d& coordinates, cv::Mat& descriptor);

 /**
  * Remove a landmark from the map
  */
  void removePoint(int ptID);

 /**
  * Update the coordinates of a landmark
  * @param[in] ptID ID of point to update
  * @param[in] new_point new coordinates of the point
  */
  void updatePoint(int ptID, cv::Point3d new_point);

 /**
  * Seta point as seen by a keyframe
  * @param[in] ptID ID of point to be set as seen
  * @param[in] kfID ID of keyframe to be set as seeing the point
  * @param[in] idx_in_kf Index of the point in the keyframe
  */
  void setPointAsSeen(int ptID, int kfID, int idx_in_kf);

 /**
  * Remove a keyframe from the map
  */
  void removeKeyframe(int kfID);


public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; //!< Pointer to PCL pointcloud object

  //! Contructors. Initialize an empty map
  Map();
  Map(ros::NodeHandle* nh);

  //! Destructor.
  ~Map();

  void reset();

 /**
  * This function is called on each new frame, it calls pnp, and the decision to create a keyframe.
  * @param[in] frame Frame to process (estimate position, and decide whether to craete a keyframe)
  * @param[out] PnP_pose result of pnp (estimated pose)
  */
  bool processFrame(Frame& frame, ucl_drone::Pose3D& PnP_pose);

 /** Set manual pose to manual_pose */
  void setManualPose(const ucl_drone::Pose3D& manual_pose);

 /** Check whether map is isInitialized
  * @return true if map is isInitialized
  */
  bool isInitialized();

 /** Update a bundle
  * Update keyframes and landmarks that were adjusted by the bundle adjustment node
  * @param[in] bundlePtr Bundle message coming from the bundle adjustment node
  */
  void updateBundle(const ucl_drone::BundleMsg::ConstPtr bundlePtr);


  void publishBenchmarkInfo(); //!< Publish information for benchamrking
  void print_benchmark_info(); //!< Print benchmark information to terminal
  void print_info();           //!< Print information of the map to terminal
  void print_landmarks(); //!< Print landmarks to terminal
  void print_keyframes(); //!< Print keyframes to terminal
};

#endif /* ucl_drone_MAP_H */
