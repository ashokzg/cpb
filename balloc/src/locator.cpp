// ROS communication
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

// Msg->OpenCV
#include <cv_bridge/CvBridge.h>
#include <image_geometry/pinhole_camera_model.h>

// OpenCV & blob detection
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <BlobResult.h>

#include <boost/thread.hpp>
#include <cmath>
#include <cstdio>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat imageA, imageB;
  bool flag;
  static const double blob_area_absolute_min_ = 60;
  static const double blob_area_absolute_max_ = 5000;
  static const double blob_compactness_ = 5;
public:
  ImageConverter()
    : it_(nh_)
  {
	  ros::Rate loop_rate(0.2);
	flag = false;
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
    while(flag == false)
    {
    	loop_rate.sleep();
    	ros::spinOnce();
    }

    ImageConverter::locator();
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    imageA = cv_ptr->image;
    imageB = imageA.clone();
    flag = true;
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  void locator()
  {
    	cv::namedWindow("Tracking");
	  int hMin, hMax, sMin, sMax, vMin, vMax;
	      hMin = 0;
	      hMax = 124;
	      sMin = 95;
	      sMax = 255;
	      vMin = 139;
	      vMax = 255;
	      cv::Mat smoothed, hsvImg, t_img;
//	      cv::createTrackbar("Hue Min", "Tracking", &hMin, 255);
//	      cv::createTrackbar("Hue Max", "Tracking", &hMax, 255);
//	      cv::createTrackbar("Sat Min", "Tracking", &sMin, 255);
//	      cv::createTrackbar("Sat Max", "Tracking", &sMax, 255);
//	      cv::createTrackbar("Val Min", "Tracking", &vMin, 255);
//	      cv::createTrackbar("Val MaX", "Tracking", &vMax, 255);
//	      //inRange(hsv, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), bw);
	while(ros::ok())
	{
		cv::Mat source = imageB;

		cv::imshow(WINDOW, imageB);
		cv::waitKey(3);
		//smoothed.create(source.rows, source.cols, source.type());
		//cvPyrMeanShiftFiltering(&source, &smoothed, 2.0, 40.0);
		GaussianBlur(source, smoothed, cv::Size(9,9), 2);
		cv::cvtColor(smoothed, hsvImg, CV_BGR2HSV);
		//cv::inRange(hsvImg,cv::Scalar(0,140,80),cv::Scalar(30,255,130),t_img);
		cv::inRange(hsvImg, cv::Scalar(hMin, sMin, vMin), cv::Scalar(hMax, sMax, vMax), t_img);
		//cv::floodFill(t_img,cv::Point(0,0),cv::Scalar(255,255,255));
//		for(int i=0;i<255;i++)
//		{
//			cv::inRange(hsvImg,cv::Scalar(0,140,80),cv::Scalar(30,255,130),t_img);
//			cv::imshow("edited", t_img);
//			if(cv::waitKey(50)!=-1)
//					std::cout<<i<<std::endl;
//		}
		CBlobResult blob;
		IplImage i_img = t_img;
		blob = CBlobResult(&i_img,NULL,0);
		int num_blobs = blob.GetNumBlobs();
		std::cout<< num_blobs <<std::endl;
	    blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, blob_area_absolute_min_, blob_area_absolute_max_);
	    blob.Filter(blob, B_EXCLUDE, CBlobGetCompactness(), B_GREATER, blob_compactness_);
	    num_blobs = blob.GetNumBlobs();
	    std::cout<< "new" <<num_blobs <<std::endl;
	    for(int i =0;i<num_blobs;i++)
	    {
	    	CBlob* bl = blob.GetBlob(i);
	    	cv::Point2d uv(CBlobGetXCenter()(*bl), CBlobGetYCenter()(*bl));
	    	cv::circle(t_img,uv,50,cv::Scalar(255,0,0),5);
	    	std::cout<<uv.x << "and" << uv.y <<std::endl;
	    }
		//blob.Filter()
		cv::imshow("edited", t_img);
		cv::waitKey(3);
    	ros::spinOnce();
	}
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
