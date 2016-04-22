/*------------------------------------------------------------------------------------------*\
   Lane Detection

   General idea and some code modified from:
   chapter 7 of Computer Vision Programming using the OpenCV Library. 
   by Robert Laganiere, Packt Publishing, 2011.

   This program is free software; permission is hereby granted to use, copy, modify, 
   and distribute this source code, or portions thereof, for any purpose, without fee, 
   subject to the restriction that the copyright notice may not be removed 
   or altered from any source or altered source distribution. 
   The software is released on an as-is basis and without any warranties of any kind. 
   In particular, the software is not guaranteed to be fault-tolerant or free from failure. 
   The author disclaims all warranties with regard to this software, any use, 
   and any consequent failure, is purely the responsibility of the user.
 
   Copyright (C) 2013 Jason Dorweiler, www.transistor.io


Notes: 

	Add up number on lines that are found within a threshold of a given rho,theta and 
	use that to determine a score.  Only lines with a good enough score are kept. 

	Calculation for the distance of the car from the center.  This should also determine
	if the road in turning.  We might not want to be in the center of the road for a turn. 
	
	Several other parameters can be played with: min vote on houghp, line distance and gap.  Some
	type of feed back loop might be good to self tune these parameters. 

	We are still finding the Road, i.e. both left and right lanes.  we Need to set it up to find the
	yellow divider line in the middle. 

	Added filter on theta angle to reduce horizontal and vertical lines. 

	Added image ROI to reduce false lines from things like trees/powerlines
\*------------------------------------------------------------------------------------------*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "linefinder.h"

#define PI 3.1415926

using namespace cv;
using namespace std;


int houghVote = 200;
int showSteps = 0;
static const std::string OPENCV_WINDOW = "Lane Detector Output";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/lane_detector/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    Mat image;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
/******************************************************************************************************/
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);
    
    image = cv_ptr->image;

    Mat gray;
	cvtColor(image,gray,CV_RGB2GRAY);
	vector<string> codes;
	Mat corners;
	findDataMatrix(gray, codes, corners);
	drawDataMatrixCodes(image, codes, corners);

	Rect roi(0,image.rows/6,image.cols-1,3*image.rows/8);// set the ROI for the image
	Mat imgROI = image(roi);

   // Canny algorithm
	Mat contours;
	Canny(imgROI,contours,80,255);
	Mat contoursInv;
	threshold(contours,contoursInv,150,255,THRESH_BINARY_INV);

   // Display Canny image
	if(showSteps)
	{
		namedWindow("Canny_Original_Image");
		imshow("Canny_Original_Image",contoursInv);
		imwrite("contours.bmp", contoursInv);
	}

 /* 
	Hough tranform for line detection with feedback
	Increase by 25 for the next frame if we found some lines.  
	This is so we don't miss other lines that may crop up in the next frame
	but at the same time we don't want to start the feed back loop from scratch. 
*/
	std::vector<Vec2f> lines;
	if (houghVote < 1 or lines.size() > 2){ // we lost all lines. reset 
		houghVote = 200; 
	}
	else{ houghVote += 25;} 
	while(lines.size() < 5 && houghVote > 0){
		HoughLines(contours,lines,1,PI/180, houghVote);
		houghVote -= 5;
	}
	std::cout << houghVote << "\n";
	Mat result(imgROI.size(),CV_8U,Scalar(255));
	imgROI.copyTo(result);

   // Draw the limes
	std::vector<Vec2f>::const_iterator itrtr= lines.begin();
	Mat hough(imgROI.size(),CV_8U,Scalar(0));
	while (itrtr!=lines.end())
	{

		float rho= (*itrtr)[0];   // first element is distance rho
		float theta= (*itrtr)[1]; // second element is angle theta
		
		//if (theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 ) { // filter to remove vertical and horizontal lines
		if (theta > 0.4 && theta < 1.52 || theta < 2.74 && theta > 1.62 ) {
			// point of intersection of the line with first row
			Point pt1(rho/cos(theta),0);        
			// point of intersection of the line with last row
			Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
			// draw a white line
			line( result, pt1, pt2, Scalar(255), 1, CV_AA); 
			line( hough, pt1, pt2, Scalar(255), 1, CV_AA);
		}

		//std::cout << "line: (" << rho << "," << theta << ")\n"; 
		++itrtr;
	}

    // Display the detected line image
	if(showSteps)
	{
		namedWindow("Detected Lines with Hough");
		imshow("Detected Lines with Hough",result);
		imwrite("hough.bmp", result);
	}

   // Create LineFinder instance
	LineFinder ld;

   // Set probabilistic Hough parameters
	ld.setLineLengthAndGap(60,2);
	ld.setMinVote(5);

   // Detect lines
	std::vector<Vec4i> li= ld.findLines(contours);
	Mat houghP(imgROI.size(),CV_8U,Scalar(0));
	ld.setShift(0);
	ld.drawDetectedLines(houghP);
	//std::cout << "First Hough" << "\n";

	if(showSteps){
		namedWindow("Detected Lines with HoughP");
		imshow("Detected Lines with HoughP", houghP);
		imwrite("houghP.bmp", houghP);
	}

   // bitwise AND of the two hough images
	bitwise_and(houghP,hough,houghP);
	Mat houghPinv(imgROI.size(),CV_8U,Scalar(0));
	Mat dst(imgROI.size(),CV_8U,Scalar(0));
	threshold(houghP,houghPinv,180,255,THRESH_BINARY_INV); // threshold and invert to black lines

	if(showSteps){
		namedWindow("Detected Lines with Bitwise");
		imshow("Detected Lines with Bitwise", houghPinv);
	}

	Canny(houghPinv,contours,150,250);
	li= ld.findLines(contours);

   // Display Canny image
	if(showSteps){
		namedWindow("Contours_Bitwise_Image");
		imshow("Contours_Bitwise_Image",contours);
		imwrite("contours.bmp", contoursInv);
	}

	   // Set probabilistic Hough parameters
	ld.setLineLengthAndGap(10,2);
	ld.setMinVote(1);
	ld.setShift(image.cols/6);
	ld.drawDetectedLines(image);
		
	std::stringstream stream;
	stream << "Lines Segments: " << lines.size();
	
	putText(image, stream.str(), Point(10,image.rows-10), 2, 0.8, Scalar(0,0,255),0);
    imshow(OPENCV_WINDOW, image);

	char key = (char) waitKey(10);
	lines.clear();    

    cv_ptr->image = image;
/*****************************************************************************************************/    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main(int argc, char* argv[]) 
{

  ros::init(argc, argv, "laneDetector");
  ImageConverter ic;
  ros::spin();
  return 0;
}



