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
    image_sub_ = it_.subscribe("/uwsim/camera1", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/buoy/output", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }
float find_distance(float radius){
    if(radius==-1){
      return -1;
    }
    float P_def = 88;           //Radius of the ball in image while calibration
    float d_def = 20;           //Distance of ball while calibration
    float l_def = 5;            //Actual Diameter of the ball
    float f = P_def * d_def / l_def;  //Apparent Focal Length of the Camera
    l_def = l_def/2;              //using radius insted of diameter of the ball
    float distance = f * l_def / radius;
    return distance;
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

  
    Mat invid, frame, frame_HSV, resG,resY,resR, framecol,resB,mask;
        frame=cv_ptr->image;
        // Convert from BGR to HSV colorspace
        
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, Scalar(48,147,0), Scalar(95,255,255), resG);
        inRange(frame_HSV, Scalar(11,144,127), Scalar(44,255,255), resY);
        inRange(frame_HSV, Scalar(126,0,0), Scalar(255,255,255), resR);
    vector<vector<Point> > contoursG;
    vector<vector<Point> > contoursY;
    vector<vector<Point> > contoursR;
   	
    medianBlur(resG, resG, 3);
    medianBlur(resY, resY, 3);
    medianBlur(resR, resR, 3);
    //erode(resY,resY,(45,45));
    //erode(resY,resY,(45,45));
    
    	findContours( resG, contoursG, RETR_TREE, CHAIN_APPROX_SIMPLE );
        findContours( resR, contoursR, RETR_TREE, CHAIN_APPROX_SIMPLE );
        findContours( resY, contoursY, RETR_TREE, CHAIN_APPROX_SIMPLE );
	int i = maxContourId(contoursG);	
        int j = maxContourId(contoursY);  
        int k = maxContourId(contoursR);
    vector<Point> ansG;
    vector<Point> ansY;
    vector<Point> ansR;
    Point2f centerG;
    Point2f centerY;
    Point2f centerR;
    float radiusG;
    float radiusY;
    float radiusR;
std_msgs::Float32MultiArray ret;
ret.data.clear();
    if(i>-1){
approxPolyDP( contoursG[i], ansG, 10, true);
minEnclosingCircle(ansG, centerG, radiusG);
circle( cv_ptr->image, centerG, radiusG, Scalar(0,255,0), 3, LINE_AA);
}
else{
radiusG=-1;
centerG.x=-1;
centerG.y=-1;
}


   if(j>-1){
 approxPolyDP( contoursY[j], ansY, 10, true);
minEnclosingCircle(ansY, centerY, radiusY);
circle( cv_ptr->image, centerY, radiusY, Scalar(0,255,255), 3, LINE_AA);
 }
  else{
radiusY=-1;
centerY.x=-1;
centerY.y=-1;
}
   if(k>-1) {
approxPolyDP( contoursR[k], ansR, 10, true);
minEnclosingCircle(ansR, centerR, radiusR);
 circle( cv_ptr->image, centerR, radiusR, Scalar(0,0,255), 3, LINE_AA);
}
else{
radiusR=-1;
centerR.x=-1;
centerR.y=-1;
}
   
float distanceG = ImageConverter::find_distance(radiusG);
float distanceR = ImageConverter::find_distance(radiusR);
float distanceY = ImageConverter::find_distance(radiusY);       
ret.data.push_back(radiusG);
ret.data.push_back(centerG.x);
ret.data.push_back(centerG.y); 
ret.data.push_back(distanceG);	
ret.data.push_back(radiusR);
ret.data.push_back(centerR.x);
ret.data.push_back(centerR.y); 
ret.data.push_back(distanceR);    
ret.data.push_back(radiusY);
ret.data.push_back(centerY.x);
ret.data.push_back(centerY.y); 
ret.data.push_back(distanceY);    
  
image_pub_.publish(ret);	
     
imshow(OPENCV_WINDOW, cv_ptr->image);
    
	
waitKey(3);


  }
};

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "image_converter");
  
  ImageConverter ic;
  ros::spin();
  return 0;
}
