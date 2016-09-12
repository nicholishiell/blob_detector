#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

#include "bupimo_msgs/BlobArray.h"
#include "bupimo_msgs/Blob.h"

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
bool DEBUG = false;


class BGR_Ranges{
public:
  float bMin, bMax;
  float gMin, gMax;
  float rMin, rMax;
};

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

void ReadCalibData(BGR_Ranges& filter1, BGR_Ranges& filter2, float& roi_center_x, float& roi_center_y){
 
  fstream calibFile;
  
  calibFile.open("/home/pi/ns_catkin_ws/calibData/center_of_roi");
  calibFile >> roi_center_x;
  calibFile >> roi_center_y;
  calibFile.close();
  
  calibFile.open("/home/pi/ns_catkin_ws/calibData/bgr_ranges");
  calibFile >> filter1.bMin;
  calibFile >> filter1.bMax;
  calibFile >> filter1.gMin;
  calibFile >> filter1.gMax;
  calibFile >> filter1.rMin;
  calibFile >> filter1.rMax;

  calibFile >> filter2.bMin;
  calibFile >> filter2.bMax;
  calibFile >> filter2.gMin;
  calibFile >> filter2.gMax;
  calibFile >> filter2.rMin;
  calibFile >> filter2.rMax;
  
  calibFile.close();

}


int main(int argc, char **argv){
  float roi_center_x = 0.;
  float roi_center_y = 0.;

  BGR_Ranges filter1;
  BGR_Ranges filter2;

  ReadCalibData(filter1, filter2, roi_center_x, roi_center_y);

  ros::init(argc, argv, "blob_detector");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);
  ros::Publisher global_pub = n.advertise<bupimo_msgs::BlobArray>("blobsGlobal", 1000);
  ros::Publisher local_pub = n.advertise<bupimo_msgs::BlobArray>("blobsLocal", 1000);

  ros::Subscriber currentHeadingSub = n.subscribe("currentHeading", 1000, CurrentHeadingCallback);

  raspicam::RaspiCam_Cv camera;
  cv::Mat rawImage;
  cv::Mat blueImage, bin1,bin2;
  cv::Mat dilateImage;
  cv::Mat erodeImage;

  cv::Mat erodeElement = getStructuringElement(MORPH_CROSS, Size( 5, 5 ), Point( 0, 0 ) );
  cv::Mat dilateElement = getStructuringElement(MORPH_RECT, Size( 9, 9 ), Point( 0, 0 ) );

  bupimo_msgs::BlobArray msgGlobal;     
  bupimo_msgs::BlobArray msgLocal;     

  //camera.set( CV_CAP_PROP_EXPOSURE, 10); // Set shutter speed 
 
  if ( !camera.open()) printf("Error opening camera\n");

 
  int imageCounter = 0;
 
  while (ros::ok()){
    
    camera.grab();
    camera.retrieve(rawImage);
   
    // Create binary mask of blue regions and store it in blueImage  
    inRange(rawImage, cv::Scalar(filter1.bMin,filter1.gMin,filter1.rMin), cv::Scalar(filter1.bMax,filter1.gMax,filter1.rMax), bin1);
    inRange(rawImage, cv::Scalar(filter2.bMin,filter2.gMin,filter2.rMin), cv::Scalar(filter2.bMax,filter2.gMax,filter2.rMax), bin2);
    blueImage = bin1 + bin2;

    
    
    erode(blueImage, erodeImage, erodeElement);
    dilate(erodeImage, dilateImage, dilateElement);

    if(DEBUG){
      cv::imwrite("binaryMaskImage.jpg", blueImage);
      cv::imwrite("rawImage.jpg", rawImage);
      cv::imwrite("dilatedImage.jpg", dilateImage);
      cv::imwrite("erodeImage.jpg", erodeImage);
    }
    
    /// Find contours   
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    findContours( dilateImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    Mat contourImage = Mat::zeros( blueImage.size(), CV_8UC3 );
    
    for( int i = 0; i< contours.size(); i++ ){
      
	double area =contourArea( contours[i],false); 

	Point * center = FindCenteroid(contours[i]);
	float cx = center->x - roi_center_x;
	float cy = center->y - roi_center_y;
	float dist = sqrt(cx*cx+cy*cy);
	
	if(area > 100. and dist > 150.){

	  float cx = center->x - roi_center_x;
	  float cy = center->y - roi_center_y;
	  
	  // Use when forward is to the right in the image
	  //float blobBearingLocal = atan2(blobY - centerY, blobX - centerX)*180./M_PI;
	  float blobBearingLocal = atan2( -1.*cx, cy)*180./M_PI;

	  float blobBearingGlobal = blobBearingLocal + currentHeading;
	  if(blobBearingGlobal < -180.) blobBearingGlobal += 360.;
	  if(blobBearingGlobal > 180.) blobBearingGlobal -= 360.;

	  bupimo_msgs::Blob aBlobGlobal;
	  bupimo_msgs::Blob aBlobLocal;

	  aBlobGlobal.bearing = blobBearingGlobal;
	  aBlobGlobal.size = area;
	  aBlobGlobal.x = cx;
	  aBlobGlobal.y = cy;

	  aBlobLocal.bearing = blobBearingLocal;
	  aBlobLocal.size = area;
	  aBlobLocal.x = cx;
	  aBlobLocal.y = cy;
	  
	  msgGlobal.blobArray.push_back(aBlobGlobal);
	  msgLocal.blobArray.push_back(aBlobLocal);
	  
	  drawContours( contourImage, contours, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point() );
	  if(DEBUG) printf("area = %f (x,y) = (%f, %f) dist = %f\n", area, cx, cy, dist);
	}
    }
    
    global_pub.publish(msgGlobal);
    local_pub.publish(msgLocal);

    msgGlobal.blobArray.clear();
    msgLocal.blobArray.clear();

    if(DEBUG){
      cv::imwrite("contourTest.jpg", contourImage);
      getchar();
    }
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  camera.release();
  
  return 0;
}
