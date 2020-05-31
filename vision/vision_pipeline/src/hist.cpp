#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <iostream> 
#include <bits/stdc++.h> 
static const std::string OPENCV_WINDOW = "Image window1";
using namespace cv;
using namespace std;

//Calibrate the camera for correct distance estimation

Mat equalizeIntensity(const Mat& inputImage)
{
    if(inputImage.channels() >= 2)
    {
        Mat temp ;
        Mat lookUpTable(1, 256, CV_8U);
        uchar* p = lookUpTable.ptr();
        for( int i = 0; i < 256; ++i)
            p[i] = saturate_cast<uchar>(pow(i / 255.0,  i < 180 ? 0.8 : 1.2) * 255.0);
        Mat res = inputImage.clone();
        
        cvtColor(res,temp,COLOR_RGB2HLS);

        vector<Mat> channels;
        split(temp,channels);
 
	    for(int i = 0; i < 1; ++i)
        {  
	        //equalizeHist(channels[2], channels[2]);
            equalizeHist(channels[1], channels[1]);
        }
    //Mat result;
        merge(channels,temp);

        cvtColor(temp,temp,COLOR_HLS2RGB);

        LUT(res, lookUpTable, res);
        float alpha = 0.5;
        float beta = 1- alpha;
        addWeighted( temp, alpha, res, beta, 0.0, temp);


        return temp;
    }
	else
	{
		Mat res;	
		equalizeHist(inputImage, res);
		return res;
	}
    return Mat();
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/uwsim/camera1", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/histogram/output", 1);

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
    //Mat rgbcolor;
    //cvtColor(cv_ptr->image,rgbcolor,cv::COLOR_BGR2RGB);
    Mat res = equalizeIntensity(cv_ptr->image);
    imshow(OPENCV_WINDOW, res);
    waitKey(3);
    sensor_msgs::ImagePtr msgret;
    cvtColor(res,res,COLOR_RGB2BGR);
    msgret = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res).toImageMsg();
    image_pub_.publish(msgret);
  }
};

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "image_converter");
  
  ImageConverter ic;
  ros::spin();
  return 0;
}
