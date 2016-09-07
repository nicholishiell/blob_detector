#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "opencv2/opencv.hpp"
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include "iostream"

using namespace std;
using namespace cv;

float currentHeading = 0.;

void CurrentHeadingCallback(const std_msgs::Float64::ConstPtr& msg){
  currentHeading = msg->data;
}

Point* FindCenteroid( Vector<Point> contour){
  Point * center = new Point();
  center->x = 0.;
  center->y = 0.;
  
  for (unsigned int i = 0; i < contour.size(); i++) {
    center->x += contour[i].x;
    center->y += contour[i].y;
  }

  float n = (float)contour.size();

  center->x /= n;
  center->y /= n;
  
  return center;
}

int main(int argc, char **argv){
  float centerX = 0.;
  float centerY = 0.;

  float bMin, bMax;
  float gMin, gMax;
  float rMin, rMax;
  
  fstream calibFile;
  
  calibFile.open("/home/pi/ns_catkin_ws/calibData/center_of_roi");
  calibFile >> centerX;
  calibFile >> centerY;
  calibFile.close();
  
  calibFile.open("/home/pi/ns_catkin_ws/calibData/bgr_ranges");
  calibFile >> bMin;
  calibFile >> bMax;
  calibFile >> gMin;
  calibFile >> gMax;
  calibFile >> rMin;
  calibFile >> rMax;
  calibFile.close();


  ros::init(argc, argv, "blob_detector");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  ros::Publisher global_pub = n.advertise<std_msgs::Float64MultiArray>("blobBearings", 1000);
  ros::Publisher local_pub = n.advertise<std_msgs::Float64MultiArray>("blobBearingsLocal", 1000);

  ros::Subscriber currentHeadingSub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);

  raspicam::RaspiCam_Cv camera;
  cv::Mat rawImage;
  cv::Mat blueImage;
  cv::Mat dilateImage;
  cv::Mat erodeImage;
  
  //camera.set( CV_CAP_PROP_EXPOSURE, 10); // Set shutter speed 100 for passive beacons)
 
  if ( !camera.open()) printf("Error opening camera\n");

  bool keepGoing = true;
  int imageCounter = 0;
  bool first = true;

  while (ros::ok() and keepGoing){
    
    camera.grab();
    camera.retrieve(rawImage);
   
    // Create binary mask of blue regions and store it in blueImage  
    inRange(rawImage, cv::Scalar(bMin,gMin,rMin), cv::Scalar(bMax,gMax,rMax), blueImage);
    cv::imwrite("binaryMaskImage.jpg", blueImage);
    cv::imwrite("rawImage.jpg", rawImage);

    Mat erodeElement = getStructuringElement(MORPH_CROSS, Size( 5, 5 ), Point( 0, 0 ) );
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size( 5, 5 ), Point( 0, 0 ) );
    
    erode(blueImage, erodeImage, erodeElement);
    dilate(erodeImage, dilateImage, dilateElement);
    
    cv::imwrite("dilatedImage.jpg", dilateImage);
    cv::imwrite("erodeImage.jpg", erodeImage);
       
    /// Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    findContours( dilateImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    /// Draw contours
    Mat contourImage = Mat::zeros( blueImage.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

	double a=contourArea( contours[i],false); 
	Point * center = FindCenteroid(contours[i]);
	
	if(a > 100.){
	  drawContours( contourImage, contours, i, color, 2, 8, hierarchy, 0, Point() );
	  float cx = center->x-centerX;
	  float cy = center->y-centerY;
	  printf("area = %f (x,y) = (%f, %f) dist = %f\n", a, cx, cy, sqrt(cx*cx+cy*cy));
	}
    }
    cv::imwrite("contourTest.jpg", contourImage);
    getchar();
    
    /*std_msgs::Float64MultiArray msgGlobal;     
    std_msgs::Float64MultiArray msgLocal;     
    
    msgGlobal.data.resize(keypoints.size()); 
    msgLocal.data.resize(keypoints.size());  
 
    for(int i = 0; i < keypoints.size(); i++){
      float blobX =  keypoints[i].pt.x;
      float blobY = keypoints[i].pt.y; 
      float blobSize = keypoints[i].size; 
	
      // Use when forward is to the right in the image
      //float blobBearingLocal = atan2(blobY - centerY, blobX - centerX)*180./M_PI;

      // Use when forward is down in the image
      float blobBearingLocal = atan2( -1.*(blobX - centerX), blobY - centerY)*180./M_PI; 

      float blobBearingGlobal = blobBearingLocal + currentHeading;
      if(blobBearingGlobal < -180.) blobBearingGlobal += 360.;
      if(blobBearingGlobal > 180.) blobBearingGlobal -= 360.;
         
      msgGlobal.data[i] = blobBearingGlobal;
      msgLocal.data[i] = blobBearingLocal;
      
      //printf("%f\t%f\n", blobBearingGlobal, blobSize);
      }
    global_pub.publish(msgGlobal);
    local_pub.publish(msgLocal);
    //printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    */
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  camera.release();
  
  return 0;
}
