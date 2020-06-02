#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 500

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

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


int main( int argc, char** argv )	{

  // 出力ファイル設定
  ofstream myfile;
  myfile.open ("results1_1.txt");

  // Parameter
  double focal = 718.8560;
  cv::Point2d pp(607.1928, 185.2157);
  
  // ファイル読み込み用
  char filename[100];

  // 初期位置・姿勢
  Mat R_f = cv::Mat::eye(3,3,CV_64F);
  Mat t_f = cv::Mat::zeros(3,1,CV_64F);
  bool read_initial_image = false; 

  // 画像&特徴点用
  Mat prevImage, currImage;
  vector<Point2f> prevFeatures, currFeatures;

  // 描画用
  namedWindow( "Road facing camera", WINDOW_AUTOSIZE );// Create a window for display.
  namedWindow( "Trajectory", WINDOW_AUTOSIZE );// Create a window for display.
  Mat traj = Mat::zeros(600, 600, CV_8UC3);
  char text[100];
  int  fontFace = FONT_HERSHEY_PLAIN;
  int  thickness = 1;  
  double fontScale = 1;
  cv::Point textOrg(10, 50);

  // Main loop
  for(int numFrame=0; numFrame < MAX_FRAME; numFrame++)	{
    sprintf(filename, "/home/yokota/data/kitti/data_odometry_gray/dataset/sequences/00/image_0/%06d.png", numFrame);
    Mat currImage = imread(filename,0);
    if (!read_initial_image) {
      featureDetection(currImage, currFeatures);
      prevImage    = currImage.clone();
      prevFeatures = currFeatures;
      read_initial_image = true;
      continue;
    }
    clock_t begin = clock();
  	vector<uchar> status;
    Mat E, R, t, mask;
  	featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
  	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
  	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
  	double scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
    if ((scale>0.1)&&(t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
      t_f = t_f + scale*(R_f*t);
      R_f = R*R_f;
    } else {
     //cout << "scale below 0.1, or incorrect translation" << endl;
    }
    
    // トラッキングしている特徴点が少ない場合は再抽出＆トラッキング
 	  if (prevFeatures.size() < MIN_NUM_FEAT)	{
 		  featureDetection(prevImage, prevFeatures);
      featureTracking(prevImage,currImage,prevFeatures,currFeatures, status);
 	  }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;

    // ターミナル出力用
    clock_t end = clock();
    const double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC * 1000.0;
    cout << "Elapsed time: " << elapsed_secs << "ms" << endl;
    
    // 描画用
    int x = int(t_f.at<double>(0)) + 300;
    int y = int(t_f.at<double>(2)) + 100;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);
    rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

    // 対象画像とトラッキング結果表示
    imshow( "Road facing camera", currImage );
    imshow( "Trajectory", traj );
    waitKey(1);
    // cout << "Scale is " << scale << endl;
    // lines for printing results
    //  myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;
  }
  return 0;
}

// Read image

// Feature detection

// Feature tracking

// Compute relative R and t