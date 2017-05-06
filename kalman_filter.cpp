// This code is used to implement Kalman Filter for tracking the bouy.
// First we will detect the bouy.
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
using namespace std;
using namespace cv;

int main(){
  VideoCapture cap("task3.mp4");
  if(!cap.isOpened()){
    cout<<"There is some load in the video and it can't be opened y'all";
    return -1;
  }
  namedWindow("Frame", WINDOW_AUTOSIZE);
  resizeWindow("Frame", 640, 480);
  int hue, saturation, value;
  hue =16;
  saturation =140 ;
  value = 106;
  createTrackbar("hue", "Frame", &hue , 180);
  createTrackbar("saturation", "Frame", &saturation, 255);
  createTrackbar("Value", "Frame", &value, 255);
  Mat_<float> measurement(2,1);
  KalmanFilter KF(4, 2, 0);
  float t = 1;
  KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,t,0,   0,1,0,t,  0,0,1,0,  0,0,0,1);
  setIdentity(KF.measurementMatrix);
  setIdentity(KF.processNoiseCov, Scalar::all(0.01));
  setIdentity(KF.measurementNoiseCov, Scalar::all(10));
  setIdentity(KF.errorCovPost, Scalar::all(1));
  vector<Point> mousev,kalmanv;
  mousev.clear();
  kalmanv.clear();

  while(cap.isOpened()){

    Mat frame,ref;
    cap>>frame>>ref;
    Mat draw(frame.rows, frame.cols, CV_8UC3, Scalar(0,0,0));
    Mat canny_output;
    int count=0;
    int sum_x=0 ; int sum_y = 0;
    blur(frame,frame, Size(41,41) );
    Canny(ref, canny_output, 50, 100);
    cvtColor(frame,frame, CV_BGR2HSV);
    for(int i=0; i<frame.rows; i++){
      for(int j=0; j<frame.cols; j++){
        if(frame.at<Vec3b>(i,j)[0]>hue && frame.at<Vec3b>(i,j)[1]<saturation && frame.at<Vec3b>(i,j)[2]>value){
          frame.at<Vec3b>(i,j)[0]=180;
          frame.at<Vec3b>(i,j)[1]=255;
          frame.at<Vec3b>(i,j)[2]=255;
          sum_x+= i;
          sum_y+= j;
          count++;
          // cout<<i<<endl<<j<<endl;
        }
        else{
          frame.at<Vec3b>(i,j)[0]=0;
          frame.at<Vec3b>(i,j)[1]=0;
          frame.at<Vec3b>(i,j)[2]=0;
        }
      }
    }

   	measurement(0)= (float)sum_x/count;
   	measurement(1) = (float)sum_y/count;
    cout<<measurement<<endl<<"This is the measurement output"<<endl;
    Mat prediction = KF.predict();
 	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
 	Mat estimated = KF.correct(measurement);

 	Point statePt(estimated.at<float>(0),estimated.at<float>(1));

 	Point measPt(measurement(0), measurement(1));
 	mousev.push_back(measPt);
    kalmanv.push_back(statePt);
    // cout<<measPt<<endl<<"This is the measured state"<<endl;
    // cout<<statePt<<endl<<"This is the estimated state"<<endl;
    /*vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
    findContours( canny_output, conto(urs, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    		Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
      	for( int i = 0; i< contours.size(); i++ )
         {
           Scalar color = Scalar( 0, 255, 0 );
           drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         }
         */
    for(int j = 0; j< mousev.size() ; j++){
    	circle(draw, mousev[j], 5,Scalar(255,0,255));
    }
    for (int i = 0; i < kalmanv.size(); i++)
      circle(draw, kalmanv[i], 5, Scalar(0,255,255));
    imshow("Frame", frame);
    // imshow("Contours", drawing);
    imshow("Reference", ref);
    imshow("drawing", draw);
    if(waitKey(100)=='q') break;
  }
}
