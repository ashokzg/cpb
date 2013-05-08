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
	tf::TransformListener tf_;
	ros::Time imTime;

	image_geometry::PinholeCameraModel model;

	bool flag;
	static const double blob_area_absolute_min_ = 300;
	static const double blob_area_absolute_max_ = 11000;
	static const double blob_compactness_ = 5;
public:
	ImageConverter() : it_(nh_)
	{
		ros::Rate loop_rate(0.2);
		flag = false;
		image_pub_ = it_.advertise("out", 1);
		//image_sub_ = it_.subscribe("/prosilica/image_rect_color", 1, &ImageConverter::imageCb, this);
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		ros::Subscriber info_sub_ = nh_.subscribe("/camera/camera_info", 1, &ImageConverter::infoCb, this);


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

	void infoCb(const sensor_msgs::CameraInfo& msg)
	{
		//model.fromCameraInfo(msg);
		//SHUTDOWN LATER #HACK
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
		int hMin, hMax, sMin, sMax, vMin, vMax,area_min;
		hMin = 0;
		//hMax = 124; // night values/???
		hMax = 255;
		sMin = 95;
		//sMin = 126;
		sMax = 255;
		vMin = 139;
		//vMin = 173;
		vMax = 255;
		area_min = 100;
		Mat smoothed, hsvImg, t_img;
		createTrackbar("blob min area","Tracking" ,&area_min ,1000);
		createTrackbar("Hue Min", "Tracking", &hMin, 255);
		createTrackbar("Hue Max", "Tracking", &hMax, 255);
		createTrackbar("Sat Min", "Tracking", &sMin, 255);
		createTrackbar("Sat Max", "Tracking", &sMax, 255);
		createTrackbar("Val Min", "Tracking", &vMin, 255);
		createTrackbar("Val MaX", "Tracking", &vMax, 255);
		while(ros::ok())
		{
			Mat source = imageB;
			const int tbl_len = 223*5, tbl_width = 112*5;
			Mat tblTransform;
			Mat newTbl;
			Point2f tlb(1251,40), trb(33,102), trt(104,667), tlt(1230,634);
			Point2f destlb(tbl_len,0), destrb(0,0), destrt(0,tbl_width), destlt(tbl_len,tbl_width);
			Point2f srcTbl[] = {tlb, trb, trt, tlt};
			//Point2f desTbl[] = {destlb, destrb, destrt, destlt};
			Point2f desTbl[] = {destrt, destlt, destlb, destrb};
			tblTransform = getPerspectiveTransform(srcTbl, desTbl);
			warpPerspective(source, newTbl, tblTransform, Size(tbl_len, tbl_width));
			Mat copy = newTbl.clone();
			rectangle(source, trb, tlt, Scalar(0,255,255),1);
			rectangle(source, tlb, trt, Scalar(0,255,255),1);

			GaussianBlur(newTbl, smoothed, Size(9,9), 4);
			cvtColor(smoothed, hsvImg, CV_BGR2HSV);
			inRange(hsvImg, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), t_img);

			CBlobResult blob;
			IplImage i_img = t_img;
			blob = CBlobResult(&i_img,NULL,0);
			int num_blobs = blob.GetNumBlobs();

			blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, area_min, blob_area_absolute_max_);
			num_blobs = blob.GetNumBlobs();

			//cout<<"Enter cornerx, cornery: ";
			//cin>>cornerx>>cornery;
			//cout<<endl;
			//circle(copy,Point2d(cornerx, cornery),5,Scalar(0,255,255),2);
			//std::string reference_frame = "/virtual_table"; // Table frame at ball_radius above the actual table plane

			//tf::StampedTransform transform;
			//tf_.waitForTransform(reference_frame, model.tfFrame(), ros::Time(0), ros::Duration(0.5));
			//tf_.lookupTransform(reference_frame, model.tfFrame(), ros::Time(0), transform);

			for(int i =0;i<num_blobs;i++)
			{
				CBlob* bl = blob.GetBlob(i);
				Point2d uv(CBlobGetXCenter()(*bl), CBlobGetYCenter()(*bl));
				//Use the width as the height
				//uv.y = bl->MinY() + (bl->MaxX() - bl->MinX()) * 0.5;
				circle(copy,uv,20,Scalar(255,0,0),5);
				stringstream ss;
				ss<<uv.x/5<<", "<<uv.y/5;
				string s = ss.str();
				int l = 1, w = 1;
				if (uv.x>tbl_len/2)
					l = -1;
				if(uv.y>tbl_width/2)
					w = -1;
				putText(copy, s, Point2d(uv.x+30*l, uv.y+30*w), FONT_HERSHEY_SIMPLEX,0.5, Scalar(0,255,255));
				cv::Point3d xyz;
				//model.projectPixelTo3dRay(uv, xyz);

				// Intersect ray with plane in virtual table frame
				//Origin of camera frame wrt virtual table frame
				//tf::Point P0 = transform.getOrigin();
				//Point at end of unit ray wrt virtual table frame
				//tf::Point P1 = transform * tf::Point(xyz.x, xyz.y, xyz.z);
				// Origin of virtual table frame
				//tf::Point V0 = tf::Point(0.0,0.0,0.0);
				// normal to the table plane
				//tf::Vector3 n(0, 0, 1);
				// finding scaling value
				//double scale = (n.dot(V0-P0))/(n.dot(P1-P0));
				//tf::Point ball_pos = P0 + (P1-P0)*scale;
				//cout <<ball_pos.x() << " " << ball_pos.y() << " " << ball_pos.z() <<endl;
			}
			imshow(WINDOW, source);
			waitKey(3);

			imshow("edited", t_img);
			waitKey(3);

			imshow("Transformed", copy);
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
