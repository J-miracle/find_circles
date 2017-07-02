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
#include "find_circles/PointArray.h"

using namespace cv;
using namespace std;

#define DATA_SIZE 600
#define R_DFNT 15
#define G_DFNT 10

float intrinsic[3][3] = {6.4436785110533890e+02, 0, 2.9885092211551677e+02, 0, 6.4885508011959678e+02, 2.5164505741385921e+02, 0, 0, 1};
float distortion[1][5] = {-4.9274063450470956e-01, 4.1714464548108104e-01, -1.4831388242744629e-03, 4.0691561547865065e-03, -2.0223623888412184e-01};

class FindCircles
{
public:
	FindCircles();
	void measurement(double &x, double &y, double u, double v, double pitch, double roll, double h);
	void revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h);
	void writeData();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;
	ros::Publisher green_pub;
	ros::Publisher red_pub;//!!!first,write a msg;second,declare in build function and use it in find_red and red_pub 

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void image_process(Mat img);
	bool find_green(Mat img, Mat &imgThresholded);
	void find_red(Mat img, Mat &imgThresholded);

	bool isGreenFound;
	int data_count = 0;
	double height;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double current_pos[2];
	cv::Point2d ardrone_pos;
	double distant[DATA_SIZE][2];
	double pose[DATA_SIZE][3];
};

FindCircles::FindCircles()
{
	img_sub = node.subscribe("/ardrone/image_raw", 1, &FindCircles::imageCallback, this);//"/ardrone/image_raw" "CamImage"
	altitude_sub = node.subscribe("/ardrone/navdata_altitude", 1, &FindCircles::altitudeCallback, this);
	navdata_sub = node.subscribe("/ardrone/navdata", 1, &FindCircles::navdataCallback, this);
	green_pub = node.advertise<geometry_msgs::Point>("green_point", 1000); //the message of centers is uncertain.
	red_pub = node.advertise<find_circles::PointArray>("red_points", 1000);
	//Mat src_img = imread("/home/wade/catkin_ws/src/find_circles/src/t.jpg", 1);//testing
	//image_process(src_img);//testing 
}

void FindCircles::writeData()
{
	/*save for evaluating the error of distant*/
	Mat distantm = Mat(DATA_SIZE, 2, CV_64FC1, distant);
	Mat posem = Mat(DATA_SIZE, 3, CV_64FC1, pose);
	ofstream f1("/home/wade/catkin_ws/src/find_circles/test_file/distant.csv");
	f1 << format(distantm, Formatter::FMT_CSV);
	ofstream f2("/home/wade/catkin_ws/src/find_circles/test_file/pose.csv");
	f2 << format(posem, Formatter::FMT_CSV);
	f1.close();
	f2.close();
}

void FindCircles::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0;
	current_pos[0] = -msg.altitude_vision/1000.0 * tan(roll);
	current_pos[1] = msg.altitude_vision/1000.0 * tan(pitch);
	//cout << "position: " << height << '\t' << current_pos[0] << '\t' << current_pos[1] << endl;
}

void FindCircles::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	roll = msg.rotX/180.0*3.1416;
	pitch = msg.rotY/180.0*3.1416;
	yaw = msg.rotZ/180.0*3.1416;
	//cout << "angles: " << roll << '\t' << pitch << '\t' << yaw << endl;
}

void FindCircles::imageCallback(const sensor_msgs::Image &msg)
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

	image_process(cv_ptr->image);
}

void FindCircles::image_process(Mat img)
{
	/*evaluation*/
	if (data_count == DATA_SIZE)
	{
		writeData();
		exit(0);
	}
	/*undistort*/
	Size image_size = img.size();
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat cameraMatrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distCoeffs = Mat(1,5,CV_32FC1,distortion);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	remap(img, img, mapx, mapy, INTER_LINEAR);
	//imshow("init_show", img);//testing
	//waitKey(0);//testing

	/* to get the actual relative position*/
	measurement(ardrone_pos.x, ardrone_pos.y, current_pos[0], current_pos[1], pitch, roll, height);
	ardrone_pos.x = -ardrone_pos.x + image_size.width/2;
	ardrone_pos.y = -ardrone_pos.y + image_size.height/2 - 45;
	cout << "ardrone_pos in image: " << ardrone_pos.x << '\t' << ardrone_pos.y << endl;
	//cv::Point center;
	//center.x = ardrone_pos.x; center.y = ardrone_pos.y;
	//cout << "center in image: " << center.x << '\t' << center.y << endl;
	circle(img, ardrone_pos, 10, Scalar(0, 255, 255), 1, 8);

	/*color abstract*/
	Mat imgThresholded = Mat(image_size, CV_8UC1);
	isGreenFound = find_green(img, imgThresholded);
	if (isGreenFound)
		ros::spin();
	else
		find_red(img, imgThresholded);
}

bool FindCircles::find_green(Mat img, Mat &imgThresholded)
{
	for (int i = 0; i < img.rows; ++i)
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
	}

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
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
		if (contourArea(*it, true) < 500)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));//testing
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}
	
	if (contours.empty())
		return false;
	Rect rb = boundingRect(Mat(contours[0]));
	geometry_msgs::Point midP;
	midP.x = 0.5*(rb.tl().x + rb.br().x);
	midP.y = 0.5*(rb.tl().y + rb.br().y);
	//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
	
	midP.x -= ardrone_pos.x;
	midP.y -= ardrone_pos.y;
	geometry_msgs::Point dist;
	revmeasurement(midP.x, midP.y, dist.x, dist.y, pitch, roll, height);

	/*publish the relative position of the circle center*/
	if (dist.x < 1 && dist.x > -1 && dist.y < 1 && dist.y > -1)
	{
		green_pub.publish(dist);
		cout << "distant: " << dist.x << ", " << dist.y << endl;
		distant[data_count][0] = dist.x;
		distant[data_count][1] = dist.y;
		pose[data_count][0] = height;
		pose[data_count][1] = pitch;
		pose[data_count][2] = roll;
		cout << data_count++ << endl;
	}

	imshow("monitor", img);//testing
	if (char(waitKey(1)) == 'o')
		destroyWindow("monitor");
	//waitKey(0);//testing
	//destroyWindow("init_show");//testing
	return true;
}

void FindCircles::find_red(Mat img, Mat &imgThresholded)
{
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

	//image operation
	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);	
	erode(imgThresholded, imgThresholded, element);
	dilate(imgThresholded, imgThresholded, element);
	imgThresholded = 255 - imgThresholded;

	//find contours
	vector<vector<Point> > contours;
	vector<Rect> rb;
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	
	//erase unqualified contours
	vector<vector<Point> >::iterator it = contours.begin();
	while(it != contours.end())
	{
		if (contourArea(*it, true) < 500)
			it = contours.erase(it);
		else
		{
			Rect r = boundingRect(Mat(*it));
			rb.push_back(r);//testing
			rectangle(img, r, Scalar(0, 255, 0), 3);//testing
			++it;
		}
	}
	
	find_circles::PointArray msg;
	vector<Rect>::iterator ir = rb.begin();
	while(ir != rb.end())
	{
		geometry_msgs::Pose2D data;
		data.x = 0.5*((*ir).tl().x + (*ir).br().x);
		data.y = 0.5*((*ir).tl().y + (*ir).br().y);
		//cout << "circle center: " << midP.x << '\t' << midP.y << endl;
	
		data.x -= ardrone_pos.x;
		data.y -= ardrone_pos.y;
		geometry_msgs::Pose2D dist;
		revmeasurement(data.x, data.y, dist.x, dist.y, pitch, roll, height);
		msg.pose.push_back(data);
	}
	red_pub.publish(msg);
	/*if (contours.empty())
	{
		cout << "Not found!\n";
		imshow("monitor", img);//testing
		if (char(waitKey(1)) == 'o')
			destroyWindow("monitor");
		ros::spin();
	}*/

	/*Rect rb = boundingRect(Mat(contours[0]));
	publish_red(rb, img);

	imshow("monitor", img);//testing
	if (char(waitKey(1)) == 'o')
		destroyWindow("monitor");
	//waitKey(0);//testing
	//destroyWindow("init_show");//testing
	*/
}

void FindCircles::measurement(double &x, double &y, double u, double v, double pitch, double roll, double h)
{
	double m = u * h / (h/cos(roll) + u*sin(roll));
	double n = v * h / (h/cos(pitch) + u*sin(pitch));
	x = 1000 * m / (1.55 * h + 0.01078);
	y = 1000 * n / (1.55 * h + 0.01078);
}
void FindCircles::revmeasurement(double x, double y, double &u, double &v, double pitch, double roll, double h)
{
	double m = (1.55 * h + 0.01078) / 1000 * x;
	double n = (1.55 * h + 0.01078) / 1000 * y;
	u = m * h/cos(roll) / (h-m*sin(roll));
	v = n * h/cos(pitch) / (h-n*sin(pitch));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_cicles");
	FindCircles fc;
	ros::spin();
	return 0;
}