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

using namespace cv;
using namespace std;

struct color_hsv
{
	int LowH; int HighH;
	int LowS; int HighS;//red(0,10,50,255,30,255)
	int LowV; int HighV;
}blue_hsv={75,130,50,255,0,255},green_hsv={35,77,50,255,40,255};

float intrinsic[3][3] = {6.4436785110533890e+02, 0, 2.9885092211551677e+02, 0, 6.4885508011959678e+02, 2.5164505741385921e+02, 0, 0, 1};
float distortion[1][5] = {-4.9274063450470956e-01, 4.1714464548108104e-01, -1.4831388242744629e-03, 4.0691561547865065e-03, -2.0223623888412184e-01};

class FindCicles
{
public:
	FindCicles();
	~FindCicles();
private:
	ros::NodeHandle node;
	ros::Subscriber img_sub;
	ros::Subscriber altitude_sub;
	ros::Subscriber navdata_sub;
	ros::Publisher midPoint_pub;

	void imageCallback(const sensor_msgs::Image &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	void navdataCallback(const ardrone_autonomy::Navdata &msg);
	void image_process(Mat img);

	double height;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double current_pos[2];
};

FindCicles::FindCicles()
{
	img_sub = node.subscribe("CamImage", 1, &FindCicles::imageCallback, this);
	altitude_sub = node.subscribe("/ardrone/navdata_altitude", 1, &FindCicles::altitudeCallback, this);
	navdata_sub = node.subscribe("/ardrone/navdata", 1, &FindCicles::navdataCallback, this);
	midPoint_pub = node.advertise<geometry_msgs::Point>("terminal", 1000); //the message of centers is uncertain.
	//Mat src_img = imread("/home/wade/catkin_ws/src/find_circles/src/t.jpg", 1);//testing
	//image_process(src_img);//testing 
}

FindCicles::~FindCicles()
{}

void FindCicles::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	height = msg.altitude_vision/1000.0 * cos(roll) * cos(pitch);
	current_pos[0] = msg.altitude_vision/1000.0 * sin(roll);
	current_pos[1] = msg.altitude_vision/1000.0 *sin(pitch);
	cout << "position: " << height << '\t' << current_pos[0] << '\t' << current_pos[1] << endl;
}

void FindCicles::navdataCallback(const ardrone_autonomy::Navdata &msg)
{
	roll = msg.rotX/180.0*3.1416;
	pitch = msg.rotY/180.0*3.1416;
	yaw = msg.rotZ/180.0*3.1416;
	cout << "angles: " << roll << '\t' << pitch << '\t' << yaw << endl;
}

void FindCicles::imageCallback(const sensor_msgs::Image &msg)
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

void FindCicles::image_process(Mat img)
{
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

	/*color abstract*/
	int iLowH = green_hsv.LowH; int iHighH = green_hsv.HighH;
	int iLowS = green_hsv.LowS; int iHighS = green_hsv.HighS;
	int iLowV = green_hsv.LowV; int iHighV = green_hsv.HighV;
	
	//RGB to HSV
	Mat imgHSV, imgThresholded;
	vector<Mat> hsvSplit;
	cvtColor(img, imgHSV, COLOR_BGR2HSV);
	
	//threshold HSV
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, imgHSV);
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
	
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
	{
		cout << "Not found!\n";
		ros::spin();
	}

	/*use midPoint in image to get the relative position*/
	Rect rb = boundingRect(Mat(contours[0]));
	geometry_msgs::Point midP;
	midP.x = 0.5*(rb.tl().x + rb.br().x);
	midP.y = 0.5*(rb.tl().y + rb.br().y);
	/*publish the relative position of the circle center*/
	midPoint_pub.publish(midP);
	cout << midP.x << ", " << midP.y << endl;

	imshow("monitor", img);//testing
	if (char(waitKey(1)) == 'o')
	{
		destroyWindow("monitor");
	}
	//waitKey(0);//testing
	//destroyWindow("init_show");//testing
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "find_cicles");
	FindCicles fc;
	ros::spin();
	return 0;
}