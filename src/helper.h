#pragma once
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

cv::Mat transformQuaternionToRotMat(double qx, double qy, double qz, double qw) {
    cv::Mat R = (cv::Mat_<double>(3,3) <<
    1.0f - 2.0f * qy * qy - 2.0f * qz * qz,
    2.0f * qx * qy + 2.0f * qw * qz,
    2.0f * qx * qz - 2.0f * qw * qy,
    2.0f * qx * qy - 2.0f * qw * qz,
    1.0f - 2.0f * qx * qx - 2.0f * qz * qz,
    2.0f * qy * qz + 2.0f * qw * qx,
    2.0f * qx * qz + 2.0f * qw * qy,
    2.0f * qy * qz - 2.0f * qw * qx,
    1.0f - 2.0f * qx * qx - 2.0f * qy * qy);
    return R;
}

void transformRotationMatrixToEuler(cv::Mat R, double &angle_x, double &angle_y, double &angle_z){
  double threshold = 0.001;

  if(abs(R.at<double>(2,1) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
    angle_x = M_PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }else if(abs(R.at<double>(2,1) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時
    angle_x = - M_PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }else{
    angle_x = asin(R.at<double>(2,1));
    angle_y = atan2(-R.at<double>(2,0), R.at<double>(2,2));
    angle_z = atan2(-R.at<double>(0,1), R.at<double>(1,1));
  }
}

double rad2deg(double rad){
  return rad * 180.0 / M_PI;
}

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
  
  string line;
  int i = 0;
  ifstream myfile ("/home/yokota/data/kitti/data_odometry_poses/dataset/poses/00.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      //cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

void getReferenceDltAngle(int frame_id, double& dlt_r_x, double& dlt_r_y, double& dlt_r_z) {
  
    string line;
    int i = 0;
    ifstream myfile ("/home/yokota/data/kitti/data_odometry_poses/dataset/poses/00.txt");
    cv::Mat R = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat R_prev = R.clone();
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    cv::Mat t_prev = t.clone();
    if (myfile.is_open()) {
        while (( getline (myfile,line) ) && (i<=frame_id)) {
            R_prev = R.clone();
            t_prev = t.clone();
            std::istringstream in(line);
            in >> R.at<double>(0,0) >> R.at<double>(0,1) >> R.at<double>(0,2) >> t.at<double>(0,0)
                >> R.at<double>(1,0) >> R.at<double>(1,1) >> R.at<double>(1,2) >> t.at<double>(1,0)
                >> R.at<double>(2,0) >> R.at<double>(2,1) >> R.at<double>(2,2) >> t.at<double>(2,0);
            ++i;
        }
        myfile.close();
    } else {
        cout << "Unable to open file";
    }

    double angle_x, angle_y, angle_z;
    double angle_x_prev, angle_y_prev, angle_z_prev;
    transformRotationMatrixToEuler(R, angle_x, angle_y, angle_z);
    transformRotationMatrixToEuler(R_prev, angle_x_prev, angle_y_prev, angle_z_prev);

    dlt_r_x = angle_x - angle_x_prev;
    dlt_r_y = angle_y - angle_y_prev;
    dlt_r_z = angle_z - angle_z_prev;
}