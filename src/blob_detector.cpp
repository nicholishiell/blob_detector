#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "opencv2/opencv.hpp"
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

using namespace std;
using namespace cv;

float currentHeading = 0.;

void CurrentHeadingCallback(const std_msgs::Float64::ConstPtr& msg){
  currentHeading = msg->data;
}

int main(int argc, char **argv){
  float centerX = 0.;
  float centerY = 0.;

  ros::init(argc, argv, "basic_float_publisher");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  ros::Publisher global_pub = n.advertise<std_msgs::Float64MultiArray>("blobBearings", 1000);
  ros::Publisher local_pub = n.advertise<std_msgs::Float64MultiArray>("blobBearingsLocal", 1000);

  ros::Subscriber currentHeadingSub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);

  raspicam::RaspiCam_Cv camera;
  cv::Mat rawImage;
  cv::Mat blueImage;
  cv::Mat blueOpenImage;
  cv::Mat keyPointsImage;
  cv::Mat calibImage;
  
  SimpleBlobDetector::Params paramsCalib;
  paramsCalib.minThreshold = 10;
  paramsCalib.maxThreshold = 255;
  paramsCalib.minDistBetweenBlobs = 5.0f;
  paramsCalib.filterByInertia = false;
  paramsCalib.filterByConvexity = false;
  paramsCalib.minConvexity = 0.75;
  paramsCalib.maxConvexity = 1.0;
  paramsCalib.filterByColor = false;
  paramsCalib.filterByCircularity = true;
  paramsCalib.minCircularity = 0.01;
  paramsCalib.maxCircularity = 1.;
  paramsCalib.filterByArea = true;
  paramsCalib.minArea = 10000.0f;
  paramsCalib.maxArea = 100000.0f;
  SimpleBlobDetector detectorCalib(paramsCalib);
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Used to find center of ROI and by extension forward direction
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 ); // Set image format
  camera.set( CV_CAP_PROP_EXPOSURE, 100); // Set shutter speed (1.3 for calib) 
  if ( !camera.open()) printf("Error opening camera\n");
    
  camera.grab();
  camera.retrieve(rawImage);
  
  // Create binary mask of dark areas and use it to calibrate.
  inRange(rawImage, cv::Scalar(0,0,0), cv::Scalar(100,15,15), calibImage);

  std::vector<KeyPoint> calibKeyPoints;
  detectorCalib.detect( calibImage, calibKeyPoints);
      
  drawKeypoints( calibImage, calibKeyPoints, keyPointsImage, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      
  cv::imwrite("calib.jpg", keyPointsImage);
  cv::imwrite("raw.jpg", rawImage);
  //for(int i = 0; i < calibKeyPoints.size(); i++)
  //printf("Center: (%f, %f) \t Size:%f\n", calibKeyPoints[i].pt.x, calibKeyPoints[i].pt.y, calibKeyPoints[i].size);
  centerX = calibKeyPoints[0].pt.x;
  centerY = calibKeyPoints[0].pt.y;

  camera.release();
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // Get ready for robot beacon detection
  camera.set( CV_CAP_PROP_EXPOSURE, 1.3); // Set shutter speed (1.3 for robot beacons)
  if ( !camera.open()) printf("Error opening camera\n");
  bool keepGoing = true;
  int imageCounter = 0;
 
  SimpleBlobDetector::Params paramsRobotBeacon;
  paramsRobotBeacon.minThreshold = 10;
  paramsRobotBeacon.maxThreshold = 255;
  paramsRobotBeacon.minDistBetweenBlobs = 50.0f;
  paramsRobotBeacon.filterByColor = false;

  paramsRobotBeacon.filterByInertia = false;
  paramsRobotBeacon.minInertiaRatio = 0.75;
  paramsRobotBeacon.maxInertiaRatio = 1.0;
  
  paramsRobotBeacon.filterByConvexity = false;
  paramsRobotBeacon.minConvexity = 0.75;
  paramsRobotBeacon.maxConvexity = 1.0;

  paramsRobotBeacon.filterByCircularity = false;
  paramsRobotBeacon.minCircularity = 0.75;
  paramsRobotBeacon.maxCircularity = 1.0;

  paramsRobotBeacon.filterByArea = true;
  paramsRobotBeacon.minArea = 150.0f;
  paramsRobotBeacon.maxArea = 100000.0f;

  SimpleBlobDetector detectorRobotBeacon(paramsRobotBeacon);
  
  while (ros::ok() and keepGoing){
    
    camera.grab();
    camera.retrieve(rawImage);
    
    // Create binary mask of blue regions and store it in blueImage

    //inRange(rawImage, cv::Scalar(100,100,0), cv::Scalar(255,255,100), blueImage);
    inRange(rawImage, cv::Scalar(15,0,0), cv::Scalar(255,50,75), blueImage);

    // Detect blobs in blueImage
    std::vector<KeyPoint> keypoints;
    detectorRobotBeacon.detect( blueImage, keypoints);

    for(int i = 0; i < keypoints.size(); i++)
      printf("Center: (%f, %f) \t Size:%f\n", keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].size);
    

    std_msgs::Float64MultiArray msgGlobal;     
    std_msgs::Float64MultiArray msgLocal;     
    
    msgGlobal.data.resize(keypoints.size()); 
    msgLocal.data.resize(keypoints.size());  
 
    for(int i = 0; i < keypoints.size(); i++){
      float blobX =  keypoints[i].pt.x;
      float blobY = keypoints[i].pt.y; 

      // Use when forward is to the right in the image
      //float blobBearingLocal = atan2(blobY - centerY, blobX - centerX)*180./M_PI;

      // Use when forward is down in the image
      float blobBearingLocal = atan2( -1.*(blobX - centerX), blobY - centerY)*180./M_PI; 

      float blobBearingGlobal = blobBearingLocal + currentHeading;
      if(blobBearingGlobal < -180.) blobBearingGlobal += 360.;
      if(blobBearingGlobal > 180.) blobBearingGlobal -= 360.;
         
      msgGlobal.data[i] = blobBearingGlobal;
      msgLocal.data[i] = blobBearingLocal;
      
      //printf("(%f,%f)\t%g\n", blobX, blobY, blobBearing*180./M_PI - currentHeading);
    }
    global_pub.publish(msgGlobal);
    local_pub.publish(msgLocal);
    //printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    
    /*ostringstream filenameRaw;
    ostringstream filenameBlue; 
    drawKeypoints( blueImage, keypoints, keyPointsImage, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    filenameBlue << "blue"<<imageCounter <<".jpg";
    //cv::imwrite(filenameBlue.str(),blueImage);
    cv::imwrite(filenameBlue.str(), keyPointsImage);
    
    filenameRaw << "raw"<<imageCounter++ <<".jpg";
    cv::imwrite(filenameRaw.str(),rawImage);
 
    /*getchar();*/
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  camera.release();
  
  return 0;
}
