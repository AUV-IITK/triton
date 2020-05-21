#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/tracking/tracking.hpp"
//#include <opencv2/core/ocl.hpp>
#include <vector>
#include <iostream> 
#include <bits/stdc++.h> 
#include <std_msgs/Float32MultiArray.h>
static const std::string OPENCV_WINDOW = "Image window1";
using namespace cv;
using namespace std;
Rect2d bboxG;
Rect2d bboxR;
Rect2d bboxY;
int i,j,k;
int a=0,count1=0;
string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
string trackerType = trackerTypes[7];
Ptr<Tracker> trackerG=TrackerCSRT::create();
Ptr<Tracker> trackerR=TrackerCSRT::create();
Ptr<Tracker> trackerY=TrackerCSRT::create();
//tracker = TrackerCSRT::create();

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
    image_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/tracker_buoy/output", 1);

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


 if(a==0){ 
    Mat invid, frame, frame_HSV, resG,resY,resR, framecol,resB,mask;
Rect2d boundRectG;
Rect2d boundRectR;
Rect2d boundRectY;
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
  //  erode(resY,resY,(45,45));
  // erode(resY,resY,(45,45));
    
    	findContours( resG, contoursG, RETR_TREE, CHAIN_APPROX_SIMPLE );
        findContours( resR, contoursR, RETR_TREE, CHAIN_APPROX_SIMPLE );
       findContours( resY, contoursY, RETR_TREE, CHAIN_APPROX_SIMPLE );
	 i = maxContourId(contoursG);	
     j = maxContourId(contoursY);  
     k = maxContourId(contoursR);
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
boundRectG = boundingRect( ansG );
bboxG=boundRectG;
trackerG->init(frame, bboxG);
//circle( cv_ptr->image, centerG, radiusG, Scalar(0,255,0), 3, LINE_AA);
}
else{
radiusG=-1;
centerG.x=-1;
centerG.y=-1;
bboxG.x=-1;
bboxG.y=-1;
}
a=1;


   if(j>-1){
 approxPolyDP( contoursY[j], ansY, 10, true);
minEnclosingCircle(ansY, centerY, radiusY);
 boundRectY = boundingRect( ansY );
bboxY=boundRectY;
trackerY->init(frame, bboxY);
//circle( cv_ptr->image, centerY, radiusY, Scalar(0,255,255), 3, LINE_AA);
 }
  else{
radiusY=-1;
centerY.x=-1;
centerY.y=-1;
bboxY.x=-1;
bboxY.y=-1;
}
   if(k>-1) {
approxPolyDP( contoursR[k], ansR, 10, true);
minEnclosingCircle(ansR, centerR, radiusR);
boundRectR = boundingRect( ansR );
bboxR=boundRectR;
trackerR->init(frame, bboxR);

 //circle( cv_ptr->image, centerR, radiusR, Scalar(0,0,255), 3, LINE_AA);
}
else{
radiusR=-1;
centerR.x=-1;
centerR.y=-1;
bboxR.x=-1;
bboxR.y=-1;
}


} 
else{
       Mat frame=cv_ptr->image;
        double timer = (double)getTickCount();
        bool okG=false,okY=false,okR=false;
        // Update the tracking result
       if(i>-1)  okG = trackerG->update(frame, bboxG);
	if(k>-1) okR = trackerR->update(frame, bboxR);
	if(j>-1) okY = trackerY->update(frame, bboxY);

        
        
        if (okG)
        {
            // Tracking success : Draw the tracked object
            rectangle(cv_ptr->image, bboxG, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Green out", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);

        }
    if (okR)
        {
            // Tracking success : Draw the tracked object
            rectangle(cv_ptr->image, bboxR, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "Red out", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);

        }
    if (okY)
        {
            // Tracking success : Draw the tracked object
            rectangle(cv_ptr->image, bboxY, Scalar( 255, 0, 0 ), 2, 1 );
        }
        else
        {
            // Tracking failure detected.
            putText(frame, "yellow out", Point(100,80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,255),2);

        }
        
        // Display tracker type on frame
        putText(cv_ptr->image, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);
        
        // Display FPS on frame
       // putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
count1++;
if(count1>100){a=0;count1=0;}
        // Display frame.
        imshow("Tracking", frame);
        imshow(OPENCV_WINDOW, cv_ptr->image);
        // Exit if ESC pressed.
        waitKey(3);
       
}
   
std_msgs::Float32MultiArray ret;
ret.data.clear();
ret.data.push_back(bboxG.x);
ret.data.push_back(bboxG.y); 
ret.data.push_back(bboxR.x);
ret.data.push_back(bboxR.y); 
ret.data.push_back(bboxY.x);
ret.data.push_back(bboxY.y); 
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
