
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

/* boris_drone */
#include <boris_drone/Pose3D.h>
#include <boris_drone/BenchmarkInfoMsg.h>
#include <boris_drone/ProcessedImageMsg.h>
#include <boris_drone/BundleMsg.h>
#include <boris_drone/TargetDetected.h>
#include <boris_drone/map/projection_2D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/read_from_launch.h>

#include <boris_drone/map/keyframe.h>
#include <boris_drone/map/frame.h>
#include <boris_drone/map/landmark.h>
#include <boris_drone/map/map_utils.h>
#include <boris_drone/map/camera.h>

/*!
 * \class Map
 * This object wraps functions to execute the mapping node
 */
class Map
{
private:
  // Some static const parameters
  static const int n_kf_for_ba = 4; //number of keyframes for bundle adjustment
  static const int threshold_inliers_new_keyframe = 35;
  static const int threshold_lost = 10;
  static const int threshold_new_keyframe = 50;
  static constexpr double threshold_new_keyframe_percentage = 0.25;

  static int point_ID_counter;//
  double threshold_kf_match;
  int    max_matches;
  double triangtime;
  int    nptstriang;

  //Nodehandle and publisher to communicate with bundle adjuster
  ros::NodeHandle* nh;
  std::string    bundle_channel;
  ros::Publisher bundle_pub;
  std::string    benchmark_channel;
  ros::Publisher benchmark_pub;

  /* bools */
  bool is_adjusting_bundle;
  bool second_keyframe_pending;
  bool use_2D_noise;
  bool use_3D_noise;
  bool no_bundle_adjustment;
  bool dlt_triangulation;
  bool midpoint_triangulation;
  double rpt2;//remove_point_threshold
  double rpt3;
  double rpt4;
  std::vector<double> BA_times_pass1;
  std::vector<double> BA_times_pass2;

  Camera camera;

  std::map<int,Landmark*> landmarks;
  std::map<int,Keyframe*> keyframes; //Map of ID to keyframe
  cv::Mat descriptors;               //!< descriptors of landmarks in OpenCV format

  ros::Time last_new_keyframe; //Time when we last added a keyframe

  std::list<Frame> queue_of_frames; //We keep a queue of frames to add best one to map
  int frame_counter; //number of frames in the queue


  /*!
   * This method computes the PnP estimation
   * @param[in]  current_frame The frame containing keypoints of the last camera observation
   * @param[out] PnP_pose      The visual pose estimation
   * @param[out] inliers       The indexes of keypoints with a correct matching
   * @param[in]  ref_keyframe  The Keyframe used to perform the estimation
   */
  int doPnP(const Frame& current_frame, boris_drone::Pose3D& PnP_pose, int& n_inliers);
  cv::Mat tvec;  //!< last translation vector (PnP estimation)
  cv::Mat rvec;  //!< last rotational vector (PnP estimation)


  int matchWithFrame(const Frame& frame, std::vector<cv::Point3f>& inliers_map_matching_points,
    std::vector<cv::Point2f>& inliers_frame_matching_points);

  /*!on keypoints between reference Keyframe and last Frame
   * @return true if a  new Keyframe is needed, false otherwise
   */
  bool keyframeNeeded(bool manual_pose_received, int n_inliers);

  void newKeyframe(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose);
  void newPairOfKeyframes(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose);
  void newPairOfKeyframes2(const Frame& frame, const boris_drone::Pose3D& pose, bool use_pose);

  void matchKeyframes(Keyframe* kf0, Keyframe* kf1);


  int getPointsForBA(std::vector<int> &kfIDs,
    std::map<int,std::map<int,int> > &points);

  void doBundleAdjustment(std::vector<int> kfIDs, bool is_first_pass);

  void targetDetectedPublisher();


  int addPoint(cv::Point3d& coordinates, cv::Mat descriptor);
  void removePoint(int ptID);
  void updatePoint(int ptID, cv::Point3d new_point);
  void setPointAsSeen(int ptID, int kfID, int idx_in_kf);
  void removeKeyframe(int kfID);


public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  //! Contructors. Initialize an empty map
  Map();
  Map(ros::NodeHandle* nh);

  //! Destructor.
  ~Map();

  void reset();

  bool processFrame(Frame& frame,boris_drone::Pose3D& PnP_pose, bool keyframeneeded);

  bool isInitialized();

  void updateBundle(const boris_drone::BundleMsg::ConstPtr bundlePtr);

  void publishBenchmarkInfo();
  void print_info();
  void print_landmarks();
  void print_keyframes();
  void getDescriptors(cv::Mat &map_descriptors);

};

#endif /* boris_drone_MAP_H */
