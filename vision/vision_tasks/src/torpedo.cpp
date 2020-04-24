#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream> 
#include <bits/stdc++.h> 
#include <std_msgs/Float32MultiArray.h>
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window1";
//static const std::string OPENCV_WINDOW2 = "Detection1";
//static const std::string OPENCV_WINDOW3 = "Detection2";
//static const std::string OPENCV_WINDOW4 = "inVideo";

int maxContourId(vector <vector<Point>> contours) {
    double maxArea = 0;
    double max2 = 0;
    int maxAreaContourId = -1;
    int ret = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = contourArea(contours.at(j));
        /*if (newArea > maxArea) {
		max2 = maxArea;
            maxArea = newArea;
		ret = maxAreaContourId;
            maxAreaContourId = j;
        } 
	else*/ if(newArea > max2 && newArea>100) {
	max2 = newArea;
	ret = j;
	}
    } 
    return ret;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher green,red;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/uwsim/camera1", 1,
      &ImageConverter::imageCb, this);
    green = nh_.advertise<std_msgs::Float32MultiArray>("/torpedo/green", 100);
    red = nh_.advertise<std_msgs::Float32MultiArray>("/torpedo/red", 100);
    namedWindow(OPENCV_WINDOW);
   // namedWindow(OPENCV_WINDOW2);
//namedWindow(OPENCV_WINDOW3);
//namedWindow(OPENCV_WINDOW4);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
//destroyWindow(OPENCV_WINDOW2);
//destroyWindow(OPENCV_WINDOW3);
//destroyWindow(OPENCV_WINDOW4);
//destroyWindow();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    Mat img;
    
    
    Mat invid, frame, frame_HSV, res, framecol, framecol2;
    int i;
    vector<Point> ans;
    Point2f center;
    float radius;
    vector<vector<Point> > contours;
    std_msgs::Float32MultiArray retGreen;
    std_msgs::Float32MultiArray retRed;
    retRed.data.clear();
    retGreen.data.clear();
    //while (count>0) {
	//img = cv_ptr->image;
	//frame = img.clone();
      
        // Convert from BGR to HSV colorspace
        cvtColor(cv_ptr->image, frame_HSV, COLOR_BGR2HSV);
	//invid = frame.clone();
        // Detect the object based on HSV Range Values
	//green
        inRange(frame_HSV, Scalar(78,222,120), Scalar(94,255,175), res);
        
	cvtColor(res, framecol, COLOR_GRAY2BGR);
   	
	medianBlur(res, res, 3);
    	findContours( res, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
	i = maxContourId(contours);	
	if(i>=0)
	{
		approxPolyDP( contours[i], ans, 10, true);
		minEnclosingCircle(ans, center, radius);
		if(radius>25)
		{
			//circle( framecol, center, 1, Scalar(0,0,255), 3, LINE_AA);
			circle( cv_ptr->image, center, 1, Scalar(0,0,255), 3, LINE_AA);
		
			//circle( framecol, center, radius, Scalar(0,0,255), 3, LINE_AA);
	    		circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, LINE_AA);
		}
		else
		{
			radius = -1;
			center.x = -1;
			center.y = -1;
		}
	}
	else
	{
		radius = -1;
		center.x = -1;
		center.y = -1;
	}
	
	//imshow(OPENCV_WINDOW2, framecol);
	//waitKey(3);
	retGreen.data.push_back(radius);
    //std_msgs::Float32 x,y;
	retGreen.data.push_back(center.x);
	retGreen.data.push_back(center.y); 
	green.publish(retGreen);

	inRange(frame_HSV, Scalar(97,0,60), Scalar(180,219,160), res);
        
	cvtColor(res, framecol2, COLOR_GRAY2BGR);
   	
	medianBlur(res, res, 3);
    	findContours( res, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
	i = maxContourId(contours);	
	if(i>=0)
	{
		approxPolyDP( contours[i], ans, 10, true);
		minEnclosingCircle(ans, center, radius);
		if(radius>25)
		{
			//circle( framecol2, center, 1, Scalar(0,0,255), 3, LINE_AA);
			circle( cv_ptr->image, center, 1, Scalar(0,0,255), 3, LINE_AA);
		
			//circle( framecol2, center, radius, Scalar(0,0,255), 3, LINE_AA);
	    		circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, LINE_AA);
		}
		else
		{
			radius = -1;
			center.x = -1;
			center.y = -1;
		}
	}
	else
	{
		radius = -1;
		center.x = -1;
		center.y = -1;
	}
	
	//imshow("Detection2", framecol2);
	
	retRed.data.push_back(radius);
    
	retRed.data.push_back(center.x);
	retRed.data.push_back(center.y); 
	red.publish(retRed);

        //imshow(OPENCV_WINDOW1, frame);
        //imshow(OPENCV_WINDOW3, framecol2);
	imshow(OPENCV_WINDOW, cv_ptr->image);
	waitKey(3);
   }
};

int main(int argc, char* argv[])
{
    //VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
  ros::init(argc, argv, "torpedo");
  
  ImageConverter ic;
  ros::spin();
  return 0;

    
}
