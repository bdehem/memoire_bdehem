// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/contrib/contrib.hpp>

/* Sparse Bundle Adjustment */
#include <cvsba/cvsba.h>

#include <iostream>
#include <fstream>

int readData(const char* filename, std::vector<cv::Point2d>& pointsImg);

void doBundleAdjustment(std::vector<cv::Point2d> ptsimg1,
                        std::vector<cv::Point2d> ptsimg2,
                        std::vector<cv::Point3d>& points3D,
                        std::vector<cv::Mat> R,
                        std::vector<cv::Mat> T);

int main(int argc, char * argv[])
{
    std::vector<cv::Point2d> ptsimg1;
    std::vector<cv::Point2d> ptsimg2;
    std::vector<cv::Point3d> points3D;
    readData("/home/bor/Desktop/Memoire/matlab/viewpt1.csv",ptsimg1);
    readData("/home/bor/Desktop/Memoire/matlab/viewpt2.csv",ptsimg2);

    points3D.resize(ptsimg1.size());
    points3D[0] = cv::Point3d(0, 0, 0);
    points3D[1] = cv::Point3d(0, 0, 1);
    points3D[2] = cv::Point3d(0, 1, 0);
    points3D[3] = cv::Point3d(0, 1, 1);
    points3D[4] = cv::Point3d(1, 0, 0);
    points3D[5] = cv::Point3d(1, 0, 1);
    points3D[6] = cv::Point3d(1, 1, 0);
    points3D[7] = cv::Point3d(1, 1, 1);

    cv::Mat R1 = (cv::Mat_< double >(3, 3) << 0.0,-1.0, 0.0,
                                              0.0, 0.0,-1.0,
                                              1.0, 0.0, 0.0);

    /*
    cv::Mat R1 = (cv::Mat_< double >(3, 3) << 1.0, 0.0, 0.0,
                                              0.0, 1.0, 0.0,
                                              0.0, 0.0, 1.0);
    */
    std::vector<cv::Mat> R;
    R.push_back(R1.t());
    R.push_back(R1.t());

    cv::Mat T1 = (cv::Mat_< double >(3, 1) << -10.0, 0.0, 0.0);
    cv::Mat T2 = (cv::Mat_< double >(3, 1) << -10.0, 0.0, 5.0);
    std::vector<cv::Mat> T;
    T.push_back(T1);
    T.push_back(T2);

    doBundleAdjustment(ptsimg1, ptsimg2, points3D, R, T);

    for (int i = 0; i < points3D.size(); i++)
    {
      printf("2DPoint %d: x = %f, y = %f\n", i, ptsimg1[i].x,
                                                ptsimg1[i].y);


      printf("3DPoint %d: x = %f, y = %f, z = %f\n", i, points3D[i].x,
                                                        points3D[i].y,
                                                        points3D[i].z);

    }

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



void doBundleAdjustment(std::vector<cv::Point2d> ptsimg1,
                        std::vector<cv::Point2d> ptsimg2,
                        std::vector<cv::Point3d>& points3D,
                        std::vector<cv::Mat> R,
                        std::vector<cv::Mat> T)
{

  // ======== INPUT DATA ===================
  std::vector<std::vector<cv::Point2d> > pointsImg;
  std::vector<std::vector<int> > visibility;
  std::vector<cv::Mat> cameraMatrix, distCoeffs;
  int NPOINTS = ptsimg1.size(); // number of 3d points
  int NCAMS   = 2; // number of cameras (viewpoints)

  // fill image projections
  pointsImg.resize(NCAMS);
  pointsImg[0] = ptsimg1;
  pointsImg[1] = ptsimg2;

  visibility.resize(NCAMS);
  distCoeffs.resize(NCAMS);
  cameraMatrix.resize(NCAMS);
  for (int i=0; i<NCAMS; i++)
  {
    visibility[i].resize(NPOINTS);
    for (int j=0; j<NPOINTS; j++)
    {
      visibility[i][j] = 1;
    }
    distCoeffs[i]   = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
    cameraMatrix[i] = (cv::Mat_<double>(3, 3) << 1, 0, 0,
                                                 0, 1, 0,
                                                 0, 0, 1 );
  }

  // ========== RUN BUNDLE ADJUSTMENT ===========
  cvsba::Sba sba;
  cvsba::Sba::Params params;

  params.type = cvsba::Sba::STRUCTURE;
  //params.type = cvsba::Sba::MOTIONSTRUCTURE;

  params.fixedIntrinsics = 5;
  params.fixedDistortion = 5;
  params.verbose = true;
  sba.setParams(params);
  printf("bout tu run sba\n");
  sba.run(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
  //cv::LevMarqSparse::bundleAdjust(points3D, pointsImg, visibility, cameraMatrix, R, T, distCoeffs);
  printf("ransba\n");
}
