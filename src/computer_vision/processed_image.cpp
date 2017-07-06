/*
 *  This file is part of boris_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  based on tuto:
 *  http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 */

#include <boris_drone/computer_vision/computer_vision.h>


// Contructor for the empty object
ProcessedImage::ProcessedImage()
{
  boris_drone::ProcessedImageMsg::Ptr msg(new boris_drone::ProcessedImageMsg());
  n_pts = 0;
}

// Constructor
// [in] msg: ROS Image message sent by the camera
// [in] pose_: Pose3D message before visual estimation to be attached with the processed image
// [in] prev: previous processed image used for keypoints tracking
// [in] use_OpticalFlowPyrLK: flag to enable keypoints tracking
ProcessedImage::ProcessedImage(const sensor_msgs::Image msg, const boris_drone::Pose3D pose_, ProcessedImage& prev)
{
  testing = false;
  n_pts = 0;
  this->pose = pose_;
  try
  {
    cv_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("boris_drone::imgproc::cv_bridge exception: %s", e.what());
    return;
  }
  // Resize the image according to the parameters in the launch file
  cv::Size size(Read::img_width(), Read::img_height());
  cv::resize(cv_img->image, cv_img->image, size);
  // Convert opencv image to ROS Image message format
  cv_img->toImageMsg(this->image);
  cv::Mat tracked_descriptors;
  std::vector<cv::KeyPoint> tracked_keypoints;
  // Use keypoints tracking between the previous and the current image processed
  if (prev.cv_img && prev.keypoints.size()>0 && prev.n_pts > 0 && false)
  {
    trackKeypoints(tracked_descriptors, tracked_keypoints, prev);
    //ROS_INFO("tracking: %lu points",tracked_keypoints.size());
    if (tracked_keypoints.size()<80)
    {
      //ROS_INFO("full detection (not enough tracked)");
      detectKeypoints(this->descriptors, this->keypoints,true);
    }
    else
    {
      cv::Mat detected_descriptors;
      std::vector<cv::KeyPoint> detected_keypoints;
      detectKeypoints(detected_descriptors, detected_keypoints,false);
      combineKeypoints(tracked_descriptors, tracked_keypoints, detected_descriptors, detected_keypoints);
    }
  }
  else
  {
    //ROS_INFO("full detection (prev is empty)");
    detectKeypoints(this->descriptors, this->keypoints, true);
  }
  n_pts = this->keypoints.size();
}

ProcessedImage::~ProcessedImage()
{
}

void ProcessedImage::trackKeypoints(cv::Mat& tracked_descriptors,
std::vector<cv::KeyPoint>& tracked_keypoints, ProcessedImage& prev)
{
  TIC(optical_flow);
  cv::Mat prevgray, gray; // If the picture is in colours, convert it to grayscale
  if (cv_img->image.channels() == 3)
  {
    cv::cvtColor(cv_img->image, gray, CV_RGB2GRAY);
    cv::cvtColor(prev.cv_img->image, prevgray, CV_RGB2GRAY);
  }
  else
  {
    prevgray = prev.cv_img->image;
    gray = cv_img->image;
  }

  // Prepare structures to receive keypoints tracking results
  std::vector<uchar> vstatus(prev.keypoints.size());
  std::vector<float> verror(prev.keypoints.size());
  std::vector<cv::Point2f> found;
  std::vector<cv::Point2f> to_find = Points(prev.keypoints);

  // Perform keypoints tracking
  cv::calcOpticalFlowPyrLK(prevgray, gray, to_find, found, vstatus, verror);
  // Copy all keypoints tracked with success
  tracked_descriptors = cv::Mat::zeros(prev.keypoints.size(), DESCRIPTOR_SIZE, CV_32F);
  int j = 0;
  double thresh = 12.0;
  for (int i = 0; i < prev.keypoints.size(); i++)
  {
    if (vstatus[i] && verror[i] < thresh)
    {
      cv::KeyPoint newKeyPoint = prev.keypoints[i];
      newKeyPoint.pt.x = found[i].x;
      newKeyPoint.pt.y = found[i].y;
      tracked_keypoints.push_back(newKeyPoint);
      for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
        tracked_descriptors.at<float>(j, k) = prev.descriptors.at<float>(i, k);
      j++;
    }
  }
  //ROS_INFO("%d points tracked",j);
}

void ProcessedImage::detectKeypoints(cv::Mat& detected_descriptors,
std::vector<cv::KeyPoint>& detected_keypoints, bool full_detection)
{
  TIC(detect);
  int nrow = cv_img->image.rows;
  int ncol = cv_img->image.cols;
  int x1 = (int)ncol*border_detec_frac;
  int y1 = (int)nrow*border_detec_frac;
  cv::Mat mask = cv::Mat::ones(nrow,ncol,CV_8UC1);
  //this rectangle leaves 1 tenth of the width left and right, same for height
  if (full_detection)
  {
    detector.detect(cv_img->image,detected_keypoints);
  }
  else
  {
    cv::Mat roi(mask,cv::Rect(x1, y1, ncol-2*x1, nrow-2*y1));
    roi = cv::Scalar(0);
    //if (testing==false)
    //{
      //testing = true;
      //std::cout << "mask = " <<std::endl;
      //std::cout << mask <<std::endl;
    //}
    detector.detect(cv_img->image, detected_keypoints, mask);
  }
  //TOC(detect, "detect detected_keypoints");
  if (detected_keypoints.size() == 0) return;
  TIC(extract);
  extractor.compute(cv_img->image, detected_keypoints, detected_descriptors);
  //TOC(extract, "descriptor extrator");
}

void ProcessedImage::combineKeypoints(cv::Mat& tracked_descriptors,  std::vector<cv::KeyPoint>& tracked_keypoints,
                                      cv::Mat& detected_descriptors, std::vector<cv::KeyPoint>& detected_keypoints)
{
  int nrow = cv_img->image.rows;
  int ncol = cv_img->image.cols;
  int x1 = (int)ncol*border_detec_frac;
  int y1 = (int)nrow*border_detec_frac;
  this->keypoints = detected_keypoints;
  this->descriptors = detected_descriptors;
  int count = 0;
  for(int i = 0; i<tracked_keypoints.size();i++)
  {
    //ROS_INFO("nrow = %d; x1 = %d; point.x = %f",nrow,x1,tracked_keypoints[i].pt.x);
    if (x1<tracked_keypoints[i].pt.x && tracked_keypoints[i].pt.x<ncol-2*x1
    &&  y1<tracked_keypoints[i].pt.y && tracked_keypoints[i].pt.y<nrow-2*y1)
    {
      this->keypoints.push_back(tracked_keypoints[i]);
      this->descriptors.push_back(tracked_descriptors.row(i));
      count++;
    }
  }
  //ROS_INFO("%d points ok from tracking",count);
}




// This function fill the message that will be sent by the computer vision node
// [out] msg: the filled message
// [in] target: the object to perform target detection
void ProcessedImage::convertToMsg(boris_drone::ProcessedImageMsg::Ptr& msg, Target target)
{
  msg->pose = this->pose;
  msg->image = this->image;

  if (this->keypoints.size() == 0)
    return;

  TIC(target);
  // Prepare structures for target detection
  std::vector< cv::DMatch > good_matches;
  std::vector< int > idxs_to_remove;
  std::vector< cv::Point2f > target_coord;
  // Perform target detection
  bool target_is_detected =
      target.detect(this->descriptors, this->keypoints, good_matches, idxs_to_remove, target_coord
#ifdef DEBUG_PROJECTION
                    ,
                    this->pose
#endif /* DEBUG_PROJECTION */
#ifdef DEBUG_TARGET
                    ,
                    cv_img->image
#endif /* DEBUG_TARGET */
                    );

  if (target_is_detected)
  {
    ROS_DEBUG("TARGET IS DETECTED");
    msg->target_detected = true;
    // Copy target center and corners position in the picture coordinates
    msg->target_points.resize(5);
    for (int i = 0; i < 5; i++)
    {
      msg->target_points[i].x = target_coord[i].x;
      msg->target_points[i].y = target_coord[i].y;
      msg->target_points[i].z = 0;
    }
  }
  else
  {
    msg->target_detected = false;
  }

  if (this->keypoints.size() - idxs_to_remove.size() <= 0)
  {
    // All keypoints are on the target
    return;
  }

  // Remove keypoints on the target
  msg->keypoints.resize(this->keypoints.size() - idxs_to_remove.size());
  ROS_DEBUG("ProcessedImage::init msg->keypoints.size()=%lu", msg->keypoints.size());
  int count = 0;
  int j = 0;
  for (unsigned i = 0; i < this->keypoints.size() && j < msg->keypoints.size() &&
                       (count < idxs_to_remove.size() || idxs_to_remove.size() == 0);
       i++)
  {
    // exclude target points from message to send (to avoid mapping of moving target)
    // very degueulasse: cv::KeyPoint =/= ucl_drone::KeyPoint, but they have some same fields with different names...
    if (idxs_to_remove.size() == 0 || i != idxs_to_remove[count])
    {
      // Copy the current keypoint position
      boris_drone::KeyPoint keypoint;
      geometry_msgs::Point point;
      point.x = (double)this->keypoints[i].pt.x;
      point.y = (double)this->keypoints[i].pt.y;

      keypoint.point = point;

      // Copy the current keypoint description
      std::vector< float > descriptor;
      descriptor.resize(DESCRIPTOR_SIZE);
      for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
      {
        descriptor[k] = this->descriptors.at< float >(i, k);
      }
      keypoint.descriptor = descriptor;

      msg->keypoints[j] = keypoint;
      j++;
    }
    if (idxs_to_remove.size() != 0 && i == idxs_to_remove[count])
    {
      // this way of doing is possible since idxs_to_remove are sorted in increasing order
      if (count < idxs_to_remove.size() - 1)
      {
        count++;
      }
    }
  }
  //TOC(target, "detect and remove target");
  ROS_DEBUG("=========== POINT (%f;%f)", msg->keypoints[msg->keypoints.size() - 1].point.x,
            msg->keypoints[msg->keypoints.size() - 1].point.y);
}
