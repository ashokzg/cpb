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

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Mat imageA, imageB;
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

    namedWindow(WINDOW);
    while(flag == false)
    {
    	loop_rate.sleep();
    	ros::spinOnce();
    }

    ImageConverter::locator();
  }

  ~ImageConverter()
  {
    destroyWindow(WINDOW);
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
    	namedWindow("Tracking");
	  int hMin, hMax, sMin, sMax, vMin, vMax;
	      hMin = 0;
	      hMax = 124;
	      sMin = 95;
	      sMax = 255;
	      vMin = 139;
	      vMax = 255;
	      Mat smoothed, hsvImg, t_img;
//	      createTrackbar("Hue Min", "Tracking", &hMin, 255);
//	      createTrackbar("Hue Max", "Tracking", &hMax, 255);
//	      createTrackbar("Sat Min", "Tracking", &sMin, 255);
//	      createTrackbar("Sat Max", "Tracking", &sMax, 255);
//	      createTrackbar("Val Min", "Tracking", &vMin, 255);
//	      createTrackbar("Val MaX", "Tracking", &vMax, 255);
//	      //inRange(hsv, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), bw);
	while(ros::ok())
	{
		Mat source = imageB;

		imshow(WINDOW, imageB);
		waitKey(3);
		//smoothed.create(source.rows, source.cols, source.type());
		//cvPyrMeanShiftFiltering(&source, &smoothed, 2.0, 40.0);
		GaussianBlur(source, smoothed, Size(9,9), 4);
		cvtColor(smoothed, hsvImg, CV_BGR2HSV);
		//inRange(hsvImg,Scalar(0,140,80),Scalar(30,255,130),t_img);
		inRange(hsvImg, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), t_img);
		floodFill(t_img,Point(0,0),Scalar(255,255,255));
//		for(int i=0;i<255;i++)
//		{
//			inRange(hsvImg,Scalar(0,140,80),Scalar(30,255,130),t_img);
//			imshow("edited", t_img);
//			if(waitKey(50)!=-1)
//					cout<<i<<endl;
//		}
		CBlobResult blob;
		IplImage i_img = t_img;
		vector<Vec3f> circles;
		HoughCircles(t_img, circles, CV_HOUGH_GRADIENT, 2, 40, 80, 40);
		blob = CBlobResult(&i_img,NULL,0);
		int num_blobs = blob.GetNumBlobs();
		cout<< num_blobs <<endl;
	    blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, blob_area_absolute_min_, blob_area_absolute_max_);
	    blob.Filter(blob, B_EXCLUDE, CBlobGetCompactness(), B_GREATER, blob_compactness_);
	    num_blobs = blob.GetNumBlobs();
	    cout<< "new" <<num_blobs <<endl;
	    for(int i =0;i<num_blobs;i++)
	    {
	    	CBlob* bl = blob.GetBlob(i);
	    	Point2d uv(CBlobGetXCenter()(*bl), CBlobGetYCenter()(*bl));
	    	//circle(t_img,uv,50,Scalar(255,0,0),5);
	    	cout<<uv.x << "and" << uv.y <<endl;
	    }

	    cout << "Size of circles :" << circles.size()<<endl;
	    Mat st; //= source.clone();
	    cvtColor(t_img, st, CV_GRAY2BGR);
	    for(size_t i = 0; i < circles.size(); i++)
	    {
	    	 Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			 int radius = cvRound(circles[i][2]);
			 // draw the circle center
			 circle(st, center, 3, Scalar(0,255,0), -1, 8, 0 );
			 // draw the circle outline
			 circle(st, center, radius, Scalar(0,0,255), 3, 8, 0 );
	    }

		//blob.Filter()
		imshow("edited", st);
		waitKey(3);
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
