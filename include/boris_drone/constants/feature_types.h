#ifndef FEATURE_TYPES_H
#define FEATURE_TYPES_H

//#define EXTRACTOR_TYPE TYPE_SIFT
#define DETECTOR_TYPE TYPE_SURF   //!< Keypoint detector used in computer_vision
#define EXTRACTOR_TYPE TYPE_SIFT  //!< Keypoint descriptor used in computer_vision and mapping

#define TYPE_SIFT 1
#define TYPE_FAST 2
#define TYPE_SURF 3
#define TYPE_SURF_128 4
#define TYPE_STAR 5
#define TYPE_BRISK 6
#define TYPE_ORB 7
#define TYPE_SURF_GPU 8
#define TYPE_FREAK 9

// DESCRIPTOR_SIZE
#if EXTRACTOR_TYPE == TYPE_SURF
#define DESCRIPTOR_SIZE 64
#elif EXTRACTOR_TYPE == TYPE_ORB
#define DESCRIPTOR_SIZE 32
#else
#define DESCRIPTOR_SIZE 128
#endif

// DIST_THRESHOLD depends in the descriptor type
#if EXTRACTOR_TYPE == TYPE_SIFT
#define DIST_THRESHOLD 250.0  //!< Max distance s.t. two features descriptions are similar
#elif EXTRACTOR_TYPE == TYPE_SURF
#define DIST_THRESHOLD 0.25  //!< Max distance s.t. two features descriptions are similar
#elif EXTRACTOR_TYPE == TYPE_ORB
#define DIST_THRESHOLD 50.0  //!< Max distance s.t. two features descriptions are similar
#else
#define DIST_THRESHOLD 200.0  //!< Max distance s.t. two features descriptions are similar
#endif

#if DETECTOR_TYPE == TYPE_SIFT
// static const cv::SIFT detector(0, 3, 0.1, 20, 3);
static const cv::SIFT detector(0, 3, 0.1, 15, 2);

#elif DETECTOR_TYPE == TYPE_SURF
// static const cv::SurfFeatureDetector detector(8000, 8, 3);
// static const cv::SurfFeatureDetector detector(4000, 6, 4, false);
// static const cv::SurfFeatureDetector detector(2000, 8, 3, false);
static const cv::SurfFeatureDetector detector(1800, 6, 3, false);

#elif DETECTOR_TYPE == TYPE_FAST
static const cv::FastFeatureDetector detector(50);

#elif DETECTOR_TYPE == TYPE_BRISK
static const cv::BRISK detector;

#elif DETECTOR_TYPE == TYPE_ORB
static const cv::OrbFeatureDetector detector(200, 1.4f, 5, 60, 2, 2, cv::ORB::HARRIS_SCORE, 60);
#endif

// Choose one descriptor in function of what is defined in boris_drone.h

#if EXTRACTOR_TYPE == TYPE_SIFT
static const cv::SiftDescriptorExtractor extractor;

#elif EXTRACTOR_TYPE == TYPE_SURF
static const cv::SurfDescriptorExtractor extractor(15000, 6, 4, false);
//static const cv::SurfDescriptorExtractor extractor(4000, 6, 4, false);

#elif EXTRACTOR_TYPE == TYPE_SURF_128
static const cv::SurfDescriptorExtractor extractor(4000, 6, 4, true);

#elif EXTRACTOR_TYPE == TYPE_ORB
static const cv::OrbDescriptorExtractor extractor;

#elif EXTRACTOR_TYPE == TYPE_FREAK
static const cv::FREAK extractor;
#endif


#endif /* FEATURE_TYPES_H */
