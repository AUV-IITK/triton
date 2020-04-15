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
    image_sub_ = it_.subscribe("/g500/camera1", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/image_converter/output_video", 100);

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
    Mat graymat;
    Mat threshed,res;
    vector<vector<Point> > contours;
    cvtColor(cv_ptr->image,graymat,cv::COLOR_BGR2GRAY);
    threshold(graymat,threshed, 150,255 , THRESH_BINARY);
    findContours( threshed, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    int i = maxContourId(contours);	
    vector<Point> ans;
    Point2f center;
    std_msgs::Float32MultiArray ret;
    //ret.layout.dim[0].label = "result";
    //ret.layout.dim[0].size = 1;
    //ret.layout.dim[0].stride = 1;
    //ret.layout.data_offset = 0;
    //ret.data = (float *)malloc(sizeof(float)*8);
    ret.data.clear();
    float radius;
    if(i>-1)
    {
    	approxPolyDP( contours[i], ans, 10, true);
    	minEnclosingCircle(ans, center, radius);
    	if(radius > 3)
		circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, LINE_AA);
    }

    else
    {
	radius = -1;
	center.x = -1;
	center.y = -1;
    }
	// Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);

    // Output modified video stream
    ret.data.push_back(radius);
    //std_msgs::Float32 x,y;
    ret.data.push_back(center.x);
    ret.data.push_back(center.y); 
    image_pub_.publish(ret);

  }
};

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "image_converter");
  
  ImageConverter ic;
  ros::spin();
  return 0;
}
