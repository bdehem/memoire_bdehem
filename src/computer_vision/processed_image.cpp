/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques, Alexandre Leclere, Boris Dehem
 *  \date 2016, 2017
 *
 *  based on tutorial:
 *  http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 */

#include <ucl_drone/computer_vision/processed_image.h>

// Contructor for the empty object
ProcessedImage::ProcessedImage()
{
  ucl_drone::ProcessedImageMsg::Ptr msg(new ucl_drone::ProcessedImageMsg());
  n_pts = 0;
}

//OF_mode = 1: use only optical flow
//OF_mode =-1: use only detection
//OF_mode = 0: use OF and detection hybrid
ProcessedImage::ProcessedImage(const sensor_msgs::Image& msg, const ucl_drone::Pose3D& pose, ProcessedImage& prev, int OF_mode, bool& made_full_detection)
{
  this->pose = pose;
  this->pose.header.stamp = msg.header.stamp;
  n_pts = 0;

  // convert ROS image to OpenCV image
  try
  {
    cv_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("ucl_drone::imgproc::cv_bridge exception: %s", e.what());
    return;
  }

  // Resize the image according to the parameters in the launch file
  cv::Size size(Read::img_width(), Read::img_height());
  cv::resize(cv_img->image, cv_img->image, size);

  // Convert opencv image to ROS Image message format
  cv_img->toImageMsg(this->image);

  int min_x, max_x, min_y, max_y;
  int nrow = cv_img->image.rows;
  int ncol = cv_img->image.cols;

  double pct_detect = 0.1;
  int thresh_x = (int) ((double)ncol*pct_detect);
  int thresh_y = (int) ((double)nrow*pct_detect);

  cv::Mat detected_descriptors;
  cv::Mat roi, mask;
  std::vector<cv::KeyPoint> detected_keypoints;
  double thetime;
  bool hyb = false;
  switch (OF_mode) {
    case 1:
      trackKeypoints(this->descriptors,this->keypoints, prev, min_x, max_x, min_y, max_y);
      if(min_x>max_x||min_y>max_y)
      {
        ROS_WARN("No tracked keypoints?");
        this->keypoints.clear();
        this->descriptors = cv::Mat();
        detectKeypoints(this->descriptors, this->keypoints, true, mask);
        made_full_detection = true;
      }
      else if(min_x<thresh_x||max_x>ncol-thresh_x||min_y<thresh_y||max_y>nrow-thresh_y)
      {
        thetime = ros::Time::now().toSec();
        mask = cv::Mat::zeros(nrow,ncol,CV_8UC1);
        if(min_x<thresh_x)
        {
          roi = mask(cv::Rect(0,min_y,min_x,max_y-min_y));
          roi = cv::Scalar(1);
          hyb = true;
        }
        if(max_x>ncol-thresh_x)
        {
          roi = mask(cv::Rect(max_x,min_y,ncol-max_x,max_y-min_y));
          roi = cv::Scalar(1);
          hyb = true;
        }
        if(min_y<thresh_y)
        {
          roi = mask(cv::Rect(min_x,0,max_x-min_x,min_y));
          roi = cv::Scalar(1);
          hyb = true;
        }
        if(max_y>nrow-thresh_y)
        {
          roi = mask(cv::Rect(min_x,max_y,max_x-min_x,nrow-max_y));
          roi = cv::Scalar(1);
          hyb = true;
        }
        detectKeypoints(detected_descriptors, detected_keypoints, false, mask);
        keypoints.insert(keypoints.end(), detected_keypoints.begin(), detected_keypoints.end());
        descriptors.push_back(detected_descriptors);
        n_pts = this->keypoints.size();
        std::cout << "\033[1;36m[TIC TOC]: " << "Hybrid" << ": " << ros::Time::now().toSec() - thetime << "\033[0m\n";
        ROS_INFO("Hybrid at %f",ros::Time::now().toSec());
      }
      return;
    case 0:
      thetime = ros::Time::now().toSec();
      trackKeypoints(this->descriptors,this->keypoints, prev, min_x, max_x, min_y, max_y);
      if(min_x>max_x||min_y>max_y)
      {
        ROS_WARN("No tracked keypoints?");
        this->keypoints.clear();
        this->descriptors = cv::Mat();
        detectKeypoints(this->descriptors, this->keypoints, true, mask);
        n_pts = this->keypoints.size();
        made_full_detection = true;
        return;
      }
      mask = cv::Mat::ones(nrow,ncol,CV_8UC1);
      roi = mask(cv::Rect(min_x,min_y,max_x-min_x,max_y-min_y));
      roi = cv::Scalar(0);
      detectKeypoints(detected_descriptors, detected_keypoints, false, mask);
      keypoints.insert(keypoints.end(), detected_keypoints.begin(), detected_keypoints.end());
      descriptors.push_back(detected_descriptors);
      n_pts = this->keypoints.size();
      std::cout << "\033[1;36m[TIC TOC]: " << "Hybrid" << ": " << ros::Time::now().toSec() - thetime << "\033[0m\n";
      ROS_INFO("Hybrid at %f",ros::Time::now().toSec());
      return;
    case -1:
      detectKeypoints(this->descriptors, this->keypoints, true, mask);
      made_full_detection = true;
      n_pts = this->keypoints.size();
      return;
    default :
      ROS_INFO("Invalid OF_mode (%d)",OF_mode);
      n_pts = 0;
  }
}

ProcessedImage::~ProcessedImage()
{
}


bool ProcessedImage::trackKeypoints(cv::Mat& tracked_descriptors,
std::vector<cv::KeyPoint>& tracked_keypoints, ProcessedImage& prev,
int& min_x, int& max_x, int& min_y, int& max_y)
{
  TIC(track);


  min_x = cv_img->image.cols + 1;  max_x = -1;
  min_y = cv_img->image.rows + 1;  max_y = -1;
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
  std::vector<uchar> vstatus(prev.n_pts);
  std::vector<float> verror(prev.n_pts);
  std::vector<cv::Point2f> found;
  std::vector<cv::Point2f> to_find = Points(prev.keypoints);

  // Perform keypoints tracking
  cv::calcOpticalFlowPyrLK(prevgray, gray, to_find, found, vstatus, verror);
  // Copy all keypoints tracked with success
  double thresh = 12.0;
  for (int i = 0; i < prev.n_pts; i++)
  {
    if (vstatus[i] && verror[i] < thresh)
    {
      cv::KeyPoint newKeyPoint = prev.keypoints[i];
      newKeyPoint.pt.x = found[i].x;
      newKeyPoint.pt.y = found[i].y;
      if      (found[i].x<min_x) min_x = (int)floor(found[i].x);
      else if (found[i].x>max_x) max_x = (int)floor(found[i].x);
      if      (found[i].y<min_y) min_y = (int)floor(found[i].y);
      else if (found[i].y>max_y) max_y = (int)floor(found[i].y);
      tracked_keypoints.push_back(newKeyPoint);
    }
  }
  //TOC_DISPLAY(track, "tracking");
  if (min_x < 0) min_x = 0;
  if (min_y < 0) min_y = 0;
  if (max_x >= cv_img->image.cols) max_x = cv_img->image.cols - 1;
  if (max_y >= cv_img->image.rows) max_y = cv_img->image.rows - 1;
  if (tracked_keypoints.size() < prev.n_pts * 0.75 || tracked_keypoints.size() < 80)
  {
    tracked_keypoints.clear();
    return false;
  }
  int j = 0;
  tracked_descriptors = cv::Mat::zeros(tracked_keypoints.size(), DESCRIPTOR_SIZE, CV_32F);
  for (int i = 0; i < prev.n_pts; i++)
  {
    if (vstatus[i] && verror[i] < thresh)
    {
      for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
        tracked_descriptors.at<float>(j, k) = prev.descriptors.at<float>(i, k);
      j++;
    }
  }
  return true;
}

void ProcessedImage::detectKeypoints(cv::Mat& detected_descriptors,
std::vector<cv::KeyPoint>& detected_keypoints, bool full_detection, cv::Mat& mask)
{
  TIC(detect);

  if (full_detection) detector.detect(cv_img->image, detected_keypoints);
  else                detector.detect(cv_img->image, detected_keypoints, mask);
  //TOC_DISPLAY(detect, "detect detected_keypoints");
  if (detected_keypoints.size() == 0) return;
  TIC(extract);
  extractor.compute(cv_img->image, detected_keypoints, detected_descriptors);
  //TOC_DISPLAY(extract, "descriptor extrator");
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
void ProcessedImage::convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target)
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
      ucl_drone::KeyPoint keypoint;
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
  //TOC_DISPLAY(target, "detect and remove target");
  ROS_DEBUG("=========== POINT (%f;%f)", msg->keypoints[msg->keypoints.size() - 1].point.x,
            msg->keypoints[msg->keypoints.size() - 1].point.y);
}
