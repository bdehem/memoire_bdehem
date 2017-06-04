// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <string>
#include <sstream>

int readData(const char* filename, std::vector<cv::Point2d>& pointsImg);

int main(int argc, char * argv[])
{
    std::cout << "Output sentence\n";
    std::vector<cv::Point2d> ptsimg1;
    std::vector<cv::Point2d> ptsimg2;
    std::vector<cv::Point3d> points3D;
    readData("/home/bor/Desktop/Memoire/matlab/viewpt1.csv",ptsimg1);
    readData("/home/bor/Desktop/Memoire/matlab/viewpt2.csv",ptsimg2);
    cv::Mat world2cam1, world2cam2, origin1, origin2,R1,R2,T1,T2,K1,K2;
    origin1 = (cv::Mat_<double>(3, 1) << 0,0,0);
    origin2 = (cv::Mat_<double>(3, 1) << 0,0,0.645);

    R1 = (cv::Mat_<double>(3, 3) <<  0,  0,  1,
                                    -1,  0,  0,
                                     0, -1,  0);
    R2 = (cv::Mat_<double>(3, 3) <<  0,  0,  1,
                                    -1,  0,  0,
                                     0, -1,  0);

    T1 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin1.at<double>(0,0),
                                    0, 1, 0, -origin1.at<double>(1,0),
                                    0, 0, 1, -origin1.at<double>(2,0));
    T2 = (cv::Mat_<double>(3, 4) << 1, 0, 0, -origin2.at<double>(0,0),
                                    0, 1, 0, -origin2.at<double>(1,0),
                                    0, 0, 1, -origin2.at<double>(2,0));

    K2 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                    0,     529.1, 182.2,
                                    0,     0,     1     );
    K1 = (cv::Mat_<double>(3, 3) << 529.1, 0    , 350.6,
                                    0,     529.1, 182.2,
                                    0,     0,     1     );

    cv::Mat cam0 = K1*R1.t()*T1;
    cv::Mat cam1 = K2*R2.t()*T2;
    cv::Mat pnts3D(4,ptsimg1.size(),CV_64F);

    cv::triangulatePoints(cam0,cam1,ptsimg1,ptsimg2,pnts3D);

    std::cout << "pt1 x = " << pnts3D.at<double>(0,0) << "\n";
    std::cout << "pt1 y = " << pnts3D.at<double>(1,0) << "\n";
    std::cout << "pt1 z = " << pnts3D.at<double>(2,0) << "\n";


    return 0;
}

int readData(const char* filename, std::vector<cv::Point2d>& pointsImg)
{
  std::ifstream myfile(filename);
  if ( !myfile )
  {
     std::cerr << "File does not exist!\n";
     return -1;
  }

  float a;
  char c;
  while (myfile >> a)
  {
    cv::Point2d pt;
    pt.x = a;
    myfile >> c;
    myfile >> a;
    pt.y = a;
    pointsImg.push_back(pt);
  }
  return 1;
}
