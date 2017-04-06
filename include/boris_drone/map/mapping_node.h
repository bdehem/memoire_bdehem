/*!
 *  \file mapping_node.h
 *  \brief This header file defines classes for the mapping node and visual pose estimation
 * (keyframe-based)
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef boris_drone_SIMPLE_MAP_H
#define boris_drone_SIMPLE_MAP_H
#define PCL_NO_PRECOMPILE

#define USE_PROFILING
#include <boris_drone/profiling.h>

/* Header files */
#include <boris_drone/boris_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

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
#include <boris_drone/TargetDetected.h>
#include <boris_drone/map/projection_2D.h>
#include <boris_drone/opencv_utils.h>
#include <boris_drone/read_from_launch.h>

#include <boris_drone/map/frame.h>
#include <boris_drone/map/keyframe.h>

/*!
 * \class MappingNode
 * This object wraps functions to execute the mapping node
 */
class MappingNode
{
private:
  ros::NodeHandle nh;

  /* Subscribers */
  ros::Subscriber processed_image_sub;
  std::string     processed_image_channel;
  ros::Subscriber reset_pose_sub;
  std::string     reset_pose_channel;
  ros::Subscriber end_reset_pose_sub;
  std::string     end_reset_pose_channel;
  ros::Subscriber strategy_sub;
  std::string     strategy_channel;

  /* Publishers */
  ros::Publisher pose_visual_pub;
  std::string    pose_visual_channel;
  ros::Publisher pose_correction_pub;
  std::string    pose_correction_channel;
  ros::Publisher target_pub;
  std::string    target_channel;

  /* Attributes */
  bool tracking_lost;  //!< true if the last visual information does not permit to estimate the drone pose
  bool do_search;      //!< parameter: if true the previous keyframe are used to perform searches
  bool stop_if_lost;   //!< parameter: if true the mapping stops when tracking_lost is true
  int  strategy;       //!< current strategy


  /* Tresholds for PnP */
  int threshold_lost;                        //!< min nber of matching keypoints for visual pose
  int threshold_new_keyframe;                //!< below this nber of matching keypoints => new keyframe
  double threshold_new_keyframe_percentage;  //!< below this pct of matching keypoints => new keyframe

  //! Callbacks
  void processedImageCb(const boris_drone::ProcessedImageMsg::ConstPtr processed_image_in);
  void resetPoseCb(const std_msgs::Empty& msg);
  void endResetPoseCb(const std_msgs::Empty& msg);
  void strategyCb(const std_msgs::Int16::ConstPtr strategyPtr);

  void resetList();

  /*!
   * This method computes the PnP estimation
   * @param[in]  current_frame The frame containing keypoint of the last camera observation
   * @param[out] PnP_pose      The visual pose estimation
   * @param[out] inliers       The indexes of keypoints with a correct matching
   * @param[in]  ref_keyframe  The Keyframe used to perform the estimation
   */
  bool doPnP(Frame current_frame, boris_drone::Pose3D& PnP_pose, std::vector<int>& inliers,
             Keyframe* ref_keyframe);

  /*!
   * This method searches among previously mapped Keyframe the closest one with the given Frame
   * @param[in]  pose          The estimated pose
   * @param[out] keyframe_ID   The identification number of the closest Keyframe
   * @param[in]  current_frame The frame object
   */
  bool closestKeyframe(const boris_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame);

  // Measure
  boris_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;  //!< s last message received


  /* Services Definition */

  cv::Mat camera_matrix_K;

  cv::Mat tvec;  //!< last translation vector (PnP estimation)
  cv::Mat rvec;  //!< last rotational vector (PnP estimation)

  Keyframe* reference_keyframe;  //!< The reference Keyframe is the last matching keyframe

  cv::Mat cam_plane_top;
  cv::Mat cam_plane_bottom;
  cv::Mat cam_plane_left;
  cv::Mat cam_plane_right;

  //! This method initializes planes defining the visible area from the camera (according to camera parameters)
  void init_planes();


  /*!
   * This method determines if a new Keyframe is needed
   * @param[in] number_of_common_keypoints Number of common keypoints between reference Keyframe and last Frame
   * @return true if a  new Keyframe is needed, false otherwise
   */
  bool newKeyframeNeeded(int number_of_common_keypoints);
  bool newKeyframeNeeded(int number_of_common_keypoints, Keyframe* reference_keyframe_candidate);
  void newReferenceKeyframe(Frame current_frame, boris_drone::Pose3D PnP_pose, bool PnP_success);

public:

  cv::FlannBasedMatcher matcher;  //!< object for descriptors comparison
  std::vector<Keyframe*> keyframes;
  std::vector<std::vector<int> > KeyframesSeeingPoint;

  bool pending_reset;  //!< true during a reset

  //! The cloud object containing 3D points
  pcl::PointCloud<pcl::PointXYZRGBSIFT>::Ptr cloud;

  cv::Mat descriptors; //!< descriptors in opencv format

  //! The visualizer object to perform projection
  boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

  //! Contructor. Initialize an empty map
  MappingNode();

  //! Destructor.
  ~MappingNode();

  void doBundleAdjustment();

  /*!
   * This method searches all keyframes containing keypoints visible frome the pose given
   * @param[in]  pose         The pose of the drone
   * @param[out] keyframes_ID Identification number of all keyframes in the map visible from the given pose
   */
  void getVisibleKeyframes(const boris_drone::Pose3D& pose,
                           std::vector<std::vector<int> >& keyframes_ID);

  /*! This method searches all keypoints visible frome the pose given
   * \param[in]  pose The pose of the drone
   * \param[out] idx  Indexes of all keypoints in the map visible from the given pose
   */
  void getVisiblePoints(const boris_drone::Pose3D& pose, std::vector<int>& idx);

  /*!
   * This method publishes a message if the last ImageProcessed contains information about the
   * target detection and add the estimated pose of the detected target in world coordinates
   */
  void targetDetectedPublisher();

  /*!
   * This method publishes a message containing the visual pose estimation
   * @param[in] poseFrame Pose published by sensors when the camera picture was received
   * @param[in] posePnP   Pose estimated with visual information and the contents of the map
   */
  void publishPoseVisual(boris_drone::Pose3D poseFrame, boris_drone::Pose3D posePnP);


  void matchWithFrame(Frame& frame, std::vector<std::vector<int> >& idx_matching_points,
                      std::vector<cv::Point3f>& keyframe_matching_points,
                      std::vector<cv::Point2f>& map_matching_points);
};

#endif /* boris_drone_SIMPLE_MAP_H */
