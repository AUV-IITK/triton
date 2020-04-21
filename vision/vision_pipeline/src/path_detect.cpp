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
static const std::string OPENCV_WINDOW = "Image window1";
using namespace cv;
using namespace std;

//Calibrate the camera for correct distance estimation

int maxContourId(vector <vector<Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } 
    } 
    return maxAreaContourId;
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher image_pub_;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/uwsim/camera1", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/path_detect", 100);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
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
    Mat cnvt;
    Mat threshed;
    vector<vector<Point> > contours;
    // cv::cvtColor(cv_ptr->image,cnvt,cv::COLOR_BGR2HSV);
    // cv::inRange(cnvt,Scalar(120,70,0),Scalar(180,255,255),threshed);
    cvtColor(cv_ptr->image,cnvt,cv::COLOR_BGR2GRAY);
    threshold(cnvt,threshed, 140,255 , THRESH_BINARY);
    findContours( threshed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    int i = maxContourId(contours);	
    float angle =-1;
    vector<Point> curve;
    Point2f center;
    float radius;
    RotatedRect minRect;
    cv::approxPolyDP( contours[i],curve,10,true);
    minEnclosingCircle(curve,center,radius);
    minRect = cv::minAreaRect( curve );
    angle = minRect.angle;

    // drawContours(cv_ptr->image,contours,i,(0,0,255));
    std_msgs::Float32MultiArray ret;
    ret.data.clear();
    
	// Update GUI Window
    imshow("mask",cnvt);
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream;
    ret.data.push_back(center.x);
    ret.data.push_back(center.y);
    ret.data.push_back(angle);
    image_pub_.publish(ret);

  }
};

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "path_detect");
  cout<<"Publishing to node"<<endl;
  ImageConverter ic;
  ros::spin();
  return 0;
}
