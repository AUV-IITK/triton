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

float norm(Point2f p){
  return sqrt( p.x*p.x + p.y*p.y);
}

float dot(Point2f a, Point2f b){
  return (a.x*b.x+a.y*b.y);
}


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
    Mat cnvt,frame,sel;
    Mat threshed;
    frame = cv_ptr->image;
    vector<vector<Point> > contours;
    
    cvtColor(frame,cnvt,cv::COLOR_BGR2HSV);
    inRange(cnvt,Scalar(0,135,40),Scalar(180,255,255),sel);
    // threshold(cnvt,threshed, 140,255 , THRESH_BINARY);
    medianBlur(sel,sel,3);
    findContours( sel, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    int i = maxContourId(contours);	
      float angle =-1;
      vector<Point2f> curve;
      Point2f center;
      float radius;
      RotatedRect minRect;

    if(i>=0){
      approxPolyDP( contours[i],curve,10,true);
      minEnclosingCircle(curve,center,radius);
      minRect = minAreaRect( curve );
      angle = minRect.angle;


      float x[4] = {curve[0].x,curve[1].x,curve[2].x,curve[3].x};
      float y[4] = {curve[0].y,curve[1].y,curve[2].y,curve[3].y};
      

      if(curve.size() == 4 ){
        Point2i a,b;
        if((pow((y[1]-y[0]),2)+pow((x[1]-x[0]),2)) > (pow((y[2]-y[1]),2)+pow((x[2]-x[1]),2))){
          a = Point2f((x[0]+x[3])/2,(y[0]+y[3])/2);
          b = Point2f((x[1]+x[2])/2,(y[1]+y[2])/2);
        }
        else{
          a = Point2f((x[0]+x[1])/2,(y[0]+y[1])/2);
          b = Point2f((x[2]+x[3])/2,(y[2]+y[3])/2);
        }

        center.x = (x[0]+x[1]+x[2]+x[3])/4;
        center.y = (y[0]+y[1]+y[2]+y[3])/4;

        Point2f c = (a+b)/2;
        Point2f t = Point2f(c.x,0);
        angle = acos(dot(Point2f(c.x-a.x,c.y-a.y),Point2f(c.x-t.x,c.y-t.y)) / norm(Point2f(c.x-a.x,c.y-a.y))/norm(Point2f(c.x-t.x,c.y-t.y))) * 180 / 3.14159;
        if(a.x<t.x){
          angle = angle*-1;
        }

        // cout<<angle<<endl;


        line(cv_ptr->image,center,Point2f(center.x,0),Scalar(0,255,0),1);
        line(cv_ptr->image,a,b,Scalar(0,255,0),1);
      }


      drawContours(cnvt,contours,i,(0,255,0));
      
    }
    else{
	  center.x = -1;
	  center.y = -1;
    angle = -1;
    }

    std_msgs::Float32MultiArray ret;
    ret.data.clear();
    
	// Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    // imshow("a",threshed);
    // imshow("b",sel);
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
  cout<<"Publishing"<<endl;
  ImageConverter ic;
  ros::spin();
  return 0;
}