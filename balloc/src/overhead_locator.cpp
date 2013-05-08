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
#include <billiards_msgs/BallState.h>
#include <billiards_msgs/TableState.h>

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
	ros::Publisher tbl_pub;
	int whitehMin, whitehMax, whitesMin, whitesMax, whitevMin, whitevMax,whitearea_min;
public:
	ImageConverter() : it_(nh_)
	{
		ros::Rate loop_rate(0.2);
		flag = false;
		image_pub_ = it_.advertise("out", 1);
		//image_sub_ = it_.subscribe("/prosilica/image_rect_color", 1, &ImageConverter::imageCb, this);
		image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		ros::Subscriber info_sub_ = nh_.subscribe("/camera/camera_info", 1, &ImageConverter::infoCb, this);
		tbl_pub = nh_.advertise<billiards_msgs::TableState>("table_state", 5, true);

		whitehMin = 0;
		//hMax = 124; // night values/???
		whitehMax = 255;
		whitesMin = 82; //THIS VALUE HAS BEEN CHANGED FOR WHITE (Here)
		//sMin = 126;
		whitesMax = 90; //THIS VALUE HAS BEEN CHANGED FOR WHITE (Here)
		whitevMin = 139;
		//vMin = 173;
		whitevMax = 255;
		whitearea_min = 100;
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


	Point2d white_locator()
	{
		namedWindow("Tracking_white");

		Mat smoothed, hsvImg, t_img;
		createTrackbar("blob min area","Tracking_white" ,&whitearea_min ,1000);
		createTrackbar("Hue Min", "Tracking_white", &whitehMin, 255);
		createTrackbar("Hue Max", "Tracking_white", &whitehMax, 255);
		createTrackbar("Sat Min", "Tracking_white", &whitesMin, 255);
		createTrackbar("Sat Max", "Tracking_white", &whitesMax, 255);
		createTrackbar("Val Min", "Tracking_white", &whitevMin, 255);
		createTrackbar("Val MaX", "Tracking_white", &whitevMax, 255);
		Mat source = imageB;
		const int tbl_len = 223*5, tbl_width = 112*5;
		Mat tblTransform;
		Mat newTbl;
		Point2f tlb(1251,40), trb(33,102), trt(104,667), tlt(1230,634);
		Point2f destlb(tbl_len,0), destrb(0,0), destrt(0,tbl_width), destlt(tbl_len,tbl_width);
		Point2f srcTbl[] = {tlb, trb, trt, tlt};
		Point2f desTbl[] = {destrt, destlt, destlb, destrb};
		tblTransform = getPerspectiveTransform(srcTbl, desTbl);
		warpPerspective(source, newTbl, tblTransform, Size(tbl_len, tbl_width));
		Mat copy1 = newTbl.clone();


		GaussianBlur(newTbl, smoothed, Size(9,9), 4);
		cvtColor(smoothed, hsvImg, CV_BGR2HSV);
		inRange(hsvImg, Scalar(whitehMin, whitesMin, whitevMin), Scalar(whitehMax, whitesMax, whitevMax), t_img);

		CBlobResult blob;
		IplImage i_img = t_img;
		blob = CBlobResult(&i_img,NULL,0);

		blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, whitearea_min, blob_area_absolute_max_);


		if(blob.GetNumBlobs() < 1)
			return Point2d(0,0);
		CBlob* bl = blob.GetBlob(0);
		Point2d uv(CBlobGetXCenter()(*bl), CBlobGetYCenter()(*bl));
		circle(copy1,uv,20,Scalar(255,0,0),5);
		stringstream ss;
		ss<<uv.x/5<<", "<<uv.y/5;
		string s = ss.str();
		int l = 1, w = 1;
		if (uv.x>tbl_len/2)
			l = -1;
		if(uv.y>tbl_width/2)
			w = -1;
		putText(copy1, s, Point2d(uv.x+30*l, uv.y+30*w), FONT_HERSHEY_SIMPLEX,0.5, Scalar(0,255,255));

		imshow("edited_white", t_img);
		waitKey(3);

		imshow("Transformed_white", copy1);
		waitKey(3);

		return uv;
	}



	Point2d cueStick_locator()
	{
		namedWindow("Tracking_stick");
		int hMin, hMax, sMin, sMax, vMin, vMax,area_min;
		hMin = 0;
		//hMax = 124; // night values/???
		hMax = 255;
		sMin = 111; //THIS VALUE HAS BEEN CHANGED FOR WHITE (Here)
		//sMin = 126;
		sMax = 160; //THIS VALUE HAS BEEN CHANGED FOR WHITE (Here)
		vMin = 145;
		//vMin = 173;
		vMax = 255;
		area_min = 100;
		Mat smoothed, hsvImg, t_img;
		createTrackbar("blob min area","Tracking_stick" ,&area_min ,10000);
		createTrackbar("Hue Min", "Tracking_stick", &hMin, 255);
		createTrackbar("Hue Max", "Tracking_stick", &hMax, 255);
		createTrackbar("Sat Min", "Tracking_stick", &sMin, 255);
		createTrackbar("Sat Max", "Tracking_stick", &sMax, 255);
		createTrackbar("Val Min", "Tracking_stick", &vMin, 255);
		createTrackbar("Val MaX", "Tracking_stick", &vMax, 255);
			Mat source = imageB;
			const int tbl_len = 223*5, tbl_width = 112*5;
			Mat tblTransform;
			Mat newTbl;
			Point2f tlb(1251,40), trb(33,102), trt(104,667), tlt(1230,634);
			Point2f destlb(tbl_len,0), destrb(0,0), destrt(0,tbl_width), destlt(tbl_len,tbl_width);
			Point2f srcTbl[] = {tlb, trb, trt, tlt};
			Point2f desTbl[] = {destrt, destlt, destlb, destrb};
			tblTransform = getPerspectiveTransform(srcTbl, desTbl);
			warpPerspective(source, newTbl, tblTransform, Size(tbl_len, tbl_width));
			Mat copy2 = newTbl.clone();


			GaussianBlur(newTbl, smoothed, Size(9,9), 4);
			cvtColor(smoothed, hsvImg, CV_BGR2HSV);
			inRange(hsvImg, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), t_img);

			CBlobResult blob;
			IplImage i_img = t_img;
			blob = CBlobResult(&i_img,NULL,0);

			blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, area_min, blob_area_absolute_max_);  //try changing the min area for stick, might filter out balls better


				CBlob* bl = blob.GetBlob(0);

				Point2d uv(CBlobGetXCenter()(*bl), CBlobGetYCenter()(*bl));

				double orientation = CBlobGetOrientation()(*bl);
				cout<<"Orientation"<<orientation<<endl;

				double MaxXatMaxY = CBlobGetMaxXatMaxY()(*bl);
				double MaxYatMinX = CBlobGetMaxYatMinX()(*bl);
				double MaxXatMinY = CBlobGetMinXatMinY()(*bl);
				double MinYatMaxX = CBlobGetMinYatMaxX()(*bl);
				circle(copy2,uv,20,Scalar(255,0,0),5);
				stringstream ss;
				ss<<uv.x/5<<", "<<uv.y/5;
				string s = ss.str();
				int l = 1, w = 1;
				if (uv.x>tbl_len/2)
					l = -1;
				if(uv.y>tbl_width/2)
					w = -1;
				putText(copy2, s, Point2d(uv.x+30*l, uv.y+30*w), FONT_HERSHEY_SIMPLEX,0.5, Scalar(0,255,255));

			imshow("edited_stick", t_img);
			waitKey(3);

			imshow("Transformed_stick", copy2);
			waitKey(3);

		return uv;
	}

	void locator()
	{
		namedWindow("Tracking");
		int hMin, hMax, sMin, sMax, vMin, vMax,area_min;
		Point2d white_coord(500,500);
		Point2d ball_coords[25];
		hMin = 0;
		//hMax = 124; // night values/???
		hMax = 255;
		sMin = 160;
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

			billiards_msgs::TableState tblState;
			white_coord = white_locator();    //Getting co-ordinates of the white ball based on seperate Saturation params
//			white_coord = cueStick_locator();    //Getting co-ordinates of the white ball based on seperate Saturation params

			billiards_msgs::BallState ball;

			ball.group_id = 0;
			ball.id = 0; //Cue ball has id 0
			ball.pocketed = false;
			ball.point.header.stamp = ros::Time::now();
			ball.point.header.frame_id = "table";
			ball.point.point.x = white_coord.x/500;
			ball.point.point.y = white_coord.y/500;
			ball.point.point.z = 0.028;    //ball radius
			tblState.balls.push_back(ball);

			Mat source = imageB;
			const int tbl_len = 223*5, tbl_width = 112*5;
			Mat tblTransform;
			Mat newTbl;
			Point2f tlb(1251,40), trb(33,102), trt(104,667), tlt(1230,634);
			Point2f destlb(tbl_len,0), destrb(0,0), destrt(0,tbl_width), destlt(tbl_len,tbl_width);
			Point2f srcTbl[] = {tlb, trb, trt, tlt};
			Point2f desTbl[] = {destrt, destlt, destlb, destrb}; //Order changed for rotation
			tblTransform = getPerspectiveTransform(srcTbl, desTbl);
			warpPerspective(source, newTbl, tblTransform, Size(tbl_len, tbl_width));
			Mat copy = newTbl.clone();
//			rectangle(source, trb, tlt, Scalar(0,255,255),1);
//			rectangle(source, tlb, trt, Scalar(0,255,255),1);

			GaussianBlur(newTbl, smoothed, Size(9,9), 4);
			cvtColor(smoothed, hsvImg, CV_BGR2HSV);
			inRange(hsvImg, Scalar(hMin, sMin, vMin), Scalar(hMax, sMax, vMax), t_img);

			CBlobResult blob;
			IplImage i_img = t_img;
			blob = CBlobResult(&i_img,NULL,0);
			int num_blobs = blob.GetNumBlobs();

			blob.Filter(blob, B_INCLUDE, CBlobGetArea(), B_INSIDE, area_min, blob_area_absolute_max_);
			num_blobs = blob.GetNumBlobs();


			for(int i =0;i<num_blobs;i++)
			{
				billiards_msgs::BallState ball;
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
				ball.group_id = 0;
				ball.id = i+1; //Start index at 1
				ball.pocketed = false;
				ball.point.header.stamp = ros::Time::now();
				ball.point.header.frame_id = "table";
				ball.point.point.x = uv.x/500;
				ball.point.point.y = uv.y/500;
				ball.point.point.z = 0.028;    //ball radius
				tblState.balls.push_back(ball);
			}
			tbl_pub.publish(tblState);
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
