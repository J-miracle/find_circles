#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/Navdata.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

/*struct color_hsv
{
	int LowH; int HighH;
	int LowS; int HighS;//red(0,10,50,255,30,255)
	int LowV; int HighV;
}blue_hsv={75,130,43,255,46,255},green_hsv={35,77,43,255,46,255},
red_hsv1{0,10,50,255,30,255},red_hsv2{155,179,43,255,46,255};*/

#define R_DFNT 60
#define G_DFNT 10

float intrinsic[3][3] = {6.4436785110533890e+02, 0, 2.9885092211551677e+02, 0, 6.4885508011959678e+02, 2.5164505741385921e+02, 0, 0, 1};
float distortion[1][5] = {-4.9274063450470956e-01, 4.1714464548108104e-01, -1.4831388242744629e-03, 4.0691561547865065e-03, -2.0223623888412184e-01};

class Test
{
public:
	Test();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);

	void image_test(Mat img);
	void altitude_test();

	double height;
	double roll;
	double pitch;
	double yaw;

	bool isgreen;
};

Test::Test()
{
	img_sub = node.subscribe("ardrone/image_raw", 1, &Test::imageCallback, this);
	altitude_sub = node.subscribe("/ardrone/navdata_altitude", 1, &Test::altitudeCallback, this);
	navdata_sub = node.subscribe("/ardrone/navdata", 1, &Test::navdataCallback, this);
	Mat src = imread("/home/wade/catkin_ws/src/position_estimate/test_file/test0.jpg");
	image_test(src);
}

void Test::imageCallback(const sensor_msgs::Image &msg)
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

	image_test(cv_ptr->image);
}

void Test::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0;
	cout << height << endl;
}
	
void Test::navdataCallback(const ardrone_autonomy::Navdata &msg)
{}

void Test::image_test(Mat img)
{

	/*undistort*/
	Size image_size = img.size();
	/*Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat cameraMatrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distCoeffs = Mat(1,5,CV_32FC1,distortion);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	remap(img, img, mapx, mapy, INTER_LINEAR);*/
	/*imshow("init", img);
	waitKey(0);*/

	Mat imgThresholded = Mat(image_size, CV_8UC1);
	/*use hsv to get the thresholded*/
	/*vector<Mat> hsvSplit;
	cvtColor(img, img, COLOR_BGR2HSV);
	split(img, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, img);*/

	/*for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp0<180&&tmp0>155) || (tmp0<=10&&tmp0>=0))
			{
				if (tmp1<255&&tmp1>45 && tmp2<255&&tmp2>50)
				{
					imgThresholded.at<uchar>(i,j) = 255;
					continue;
				}
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}*/

	/*for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if (tmp0<80&&tmp0>35)
			{
				if (tmp1<255&&tmp1>45 && tmp2<255&&tmp2>50)
				{
					imgThresholded.at<uchar>(i,j) = 255;
					continue;
				}
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}*/

	/*use RGB to get the thersholded*/
	/*for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp1-tmp0)>=G_DFNT && (tmp1-tmp2)>=G_DFNT)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}*/

	for (int i = 0; i < img.rows; ++i)
	{
		for (int j = 0; j < img.cols; ++j)
		{
			int tmp0 = img.at<Vec3b>(i,j)[0];
			int tmp1 = img.at<Vec3b>(i,j)[1];
			int tmp2 = img.at<Vec3b>(i,j)[2];
			if ((tmp2-tmp1)>=R_DFNT && (tmp2-tmp0)>=R_DFNT)
			{
				imgThresholded.at<uchar>(i,j) = 255;
				continue;
			}
			imgThresholded.at<uchar>(i,j) = 0;
		}
	}

	imshow("threshold", imgThresholded);
	waitKey(0);
	
	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;

	//find contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 200)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));//testing
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}
	imshow("img", img);
	waitKey(0);
	/*waitKey(0);
	destroyWindow("threshold");
	destroyWindow("init");
	destroyWindow("img");*/
}

void Test::altitude_test()
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	Test t;
	ros::spin();
	return 0;
}