
ProcessedImage::ProcessedImage(const sensor_msgs::Image msg, const boris_drone::Pose3D pose_,
                               ProcessedImage& prev, bool use_OpticalFlowPyrLK)
{
  last_number_of_keypoints = 0;
  ROS_DEBUG("ProcessedImage::init");
  this->pose = pose_;

  // convert ROS image to OpenCV image
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

  // Use keypoints tracking between the previous and the current image processed
  if (use_OpticalFlowPyrLK && prev.cv_img && prev.keypoints.size() > 0 &&
      last_number_of_keypoints != 0)  // performs only if the previous image
                                                      // processed is not empty and if keypoints
                                                      // were detected
  {
    ROS_DEBUG("use_OpticalFlowPyrLK");
    TIC(optical_flow);
    bool OF_success = false;  // flag which indicates the success of keypoints tracking

    // If the picture is in colours, convert it to grayscale
    cv::Mat prevgray, gray;
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
    std::vector< uchar > vstatus(prev.keypoints.size());
    std::vector< float > verror(prev.keypoints.size());

    std::vector< cv::Point2f > found;
    std::vector< cv::Point2f > to_find = Points(prev.keypoints);

    // Perform keypoints tracking
    cv::calcOpticalFlowPyrLK(prevgray, gray, to_find, found, vstatus, verror);

    // Copy all keypoints tracked with success
    double thres = 12.0;

    for (int i = 0; i < prev.keypoints.size(); i++)
    {
      if (vstatus[i] && verror[i] < thres)
      {
        cv::KeyPoint newKeyPoint = prev.keypoints[i];
        newKeyPoint.pt.x = found[i].x;
        newKeyPoint.pt.y = found[i].y;
        this->keypoints.push_back(newKeyPoint);
      }
    }

    // Determine if enough keypoints were detected or if new keypoints detection is needed
    OF_success = this->keypoints.size() > last_number_of_keypoints * 0.75 &&
                 this->keypoints.size() > 80;
    //TOC(optical_flow, "optical flow tracking");

    if (OF_success)  // then simply copy all descriptors previously computed
    {
      this->descriptors = cv::Mat::zeros(this->keypoints.size(), DESCRIPTOR_SIZE, CV_32F);
      for (int i = 0, j = 0; i < prev.keypoints.size(); i++)
      {
        if (vstatus[i] && verror[i] < thres)
        {
          for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
          {
            this->descriptors.at< float >(j, k) = prev.descriptors.at< float >(i, k);
          }
          j++;
        }
      }
      // KEYPOINTS TRACKING SUCCESS, no more computation needed
      return;
    }
    else
    {
      this->keypoints.clear();
    }
  }

  ROS_DEBUG("=== OPTICAL FLOW NOT SUCCESSFUL ===");
  // Keypoints detection for current image
  TIC(detect);
  detector.detect(cv_img->image, this->keypoints);
  //TOC(detect, "detect keypoints");
  ROS_DEBUG("ProcessedImage::init this->keypoints.size()=%lu", this->keypoints.size());
  last_number_of_keypoints = this->keypoints.size();
  if (this->keypoints.size() == 0)
  {
    return;
  }
  TIC(extract);
  // Perform keypoints description
  extractor.compute(cv_img->image, this->keypoints, this->descriptors);
  //TOC(extract, "descriptor extrator");
  ROS_DEBUG("end ProcessedImage::init");
}

// Constructor (no optical flow)
// [in] msg: ROS Image message sent by the camera
// [in] pose_: Pose3D message before visual estimation to be attached with the processed image
ProcessedImage::ProcessedImage(const sensor_msgs::Image msg, const boris_drone::Pose3D pose_)
{
  last_number_of_keypoints = 0;
  ROS_DEBUG("ProcessedImage::init");
  this->pose = pose_;

  // convert ROS image to OpenCV image
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

  // Keypoints detection for current image
  TIC(detect);
  detector.detect(cv_img->image, this->keypoints);
  //TOC(detect, "detect keypoints");
  ROS_DEBUG("ProcessedImage::init this->keypoints.size()=%lu", this->keypoints.size());
  last_number_of_keypoints = this->keypoints.size();
  if (this->keypoints.size() == 0)
  {
    return;
  }
  TIC(extract);
  // Perform keypoints description
  extractor.compute(cv_img->image, this->keypoints, this->descriptors);
  //TOC(extract, "descriptor extrator");
  ROS_DEBUG("end ProcessedImage::init");
}
