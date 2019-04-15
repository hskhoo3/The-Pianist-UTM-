#include "ros/ros.h"
#include <cmath>
#include <math.h> 
#include <opencv2/opencv.hpp>
#include "opencv2/gpu/gpu.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Range.h"

#define PI 3.14159265

ros::NodeHandle n;

void parkMissionStarts(const std_msgs::Bool::ConstPtr& msg)
{
	parking_Mission parkingCtrl;
}

class parking_mission()
{
	cv::cuda::GpuMat d_edge, d_image, d_frame_hsv, d_intermediate,d_result, d_imagec;
	cv::cuda::GpuMat whiteColFilter, yellowColFilter, 
	cv::cuda::GpuMat d_frame_shsv[3];
	cv::cuda::GpuMat d_thresc[3];
	
	cv::Mat h_image, h_edge;
	
	cv_bridge::CvImagePtr cv_ptr;
	
	cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge;
	Ptr<cuda::HoughSegmentDetector> hough;
	
	std::vector<cv::Vec4i> whiteLines, yellowLines;
	
	float32 sonarLeftDist, sonarRightDist;
	
	bool lower_obstacle = false;
	bool higher_obstacle = false;
	bool whiteLineDetect = false;
	bool yellowLineDetect = false;
	
	int whiteHSVLower[3] = {0,0,200};
	int yellowHSVLower[3] = {27,75,163};
	
	enum parkState 
	{
		reverseCurve, reverseStraight
	}
	
	ros::Publisher turtlemove_pub;
	
	void parking_mission::mainProcessing(const sensor_msgs::Image::ConstPtr& msg);
	cv::cuda::GpuMat parking_mission::filterCol(cv::cuda::GpuMat rawImage, int threshld[]);
	std::vector<cv::Vec4i> parkingMission::detectLine(cv::cuda::GpuMat colImage);
	void checkLeftSonar(const sensor_msgs::Range::ConstPtr& msg);
	void checkRightSonar(const sensor_msgs::Range::ConstPtr& msg);
	void parkingStarts(parkState state);
	void turtlemove(double linear, double angular);
	void checkLidarDist(const sensor_msgs::LaserScan::ConstPtr& msg);
};
		
parking_mission::parking_mission(void)
{
	ros::Subscriber lidarSub = n.subscribe('/scan', 1, checkLidarDistance);
	ros::Subscriber leftSonarSub = n.subscribe('/sonar/left', 1, checkLeftSonar);
	ros::Subscriber RightSonarSub = n.subscribe('/sonar/right', 1, checkRightSonar);
	ros::Subscriber imageSub = n.subscribe('/camera/image/compressed_fisheye', 1, mainProcessing);
	ros:spin();
	
	turtlemove_pub = n.advertise<geometry_msgs::Twist>('/cmd_vel',1);
	
	canny_edge = cv::cuda::createCannyEdgeDetector(0, 360.0, 3, false);
	hough = cuda::createHoughSegmentDetector(1.0f, (float) (CV_PI / 180.0f), 50, 5);
}

void parking_mission::mainProcessing(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


	d_image.upload(cv_ptr->image);

	whiteColFilter = filterCol(d_image, whiteHSVLower);
	yellowColFilter = filterCol(d_image, yellowHSVLower);
	
	whiteLines = detectLine(whiteColFilter);
	yellowLines = detectLine(yellowColFilter);
	
	if (! whiteLines.empty())
	{
		for (int i=0;i<whiteLines.size();i++)
		{
			cv::Vec4i line_pointw = whiteLines[i];
			
			double degree = atan2((line_pointw[3]-line_pointw[1])/(line_pointw[2]-line_pointw[0]));
			
			if (degree<5 && degree>-20)
			{
				whiteLineDetect = true;
			}
			
			else
			{
				whiteLineDetect = false;
			}
		}
	}
	
	if (! yellowLines.empty())
	{
		for (int i=0;i<yellowLines.size();i++)
		{
			cv::Vec4i line_pointy = whiteLines[i];
			
			double degree = atan2((line_pointy[3]-line_pointy[1])/(line_pointy[2]-line_pointy[0]));
			
			if (degree<5 && degree>-20)
			{
				yellowLineDetect = true;
			}
			
			else
			{
				yellowLineDetect = false;
			}
		}
	}
	
	if (whiteLineDetect == true)
	{
		turtlemove(0.06,0.00);
	}
	
	else
	{
		if (higher_obstacle == true || lower_obstacle == true)
		{
			turlemove(0.06,0.00);
		}
		
		else
		{
			parkState currParkState = reverseCurve;
			parkingStarts(currParkState);
		}	
	}
	
	if (yellowLineDetect == true)
	{
		parkState currParkState = reverseStraight;
		parkingStarts(currParkState);
	}
}

cv::cuda::GpuMat parking_mission::filterCol(cv::cuda::GpuMat rawImage, int threshld[])
{
	cv::cuda::cvtColor(rawImage, d_frame_hsv, COLOR_BGR2HSV);
	cv::cuda::split(d_frame_hsv, d_frame_shsv);
	
	cv::cuda::threshold(d_frame_shsv[0], d_thresc[0], threshold[0], 0, THRESH_BINARY);
	cv::cuda::threshold(d_frame_shsv[1], d_thresc[1], threshold[1], 0, THRESH_BINARY);
	cv::cuda::threshold(d_frame_shsv[2], d_thresc[2], threshold[2], 200, THRESH_BINARY);
	
	cv::cuda::bitwise_and(d_thresc[0], d_thresc[1],d_intermediate);
	cv::cuda::bitwise_and(d_intermediate, d_thresc[2], d_result);
	
	return d_result;
}

std::vector<cv::Vec4i> parkingMission::detectLine(cv::cuda::GpuMat colImage)
{
	canny_edge->detect(colImage, d_edge);

	hough->detect(d_edge, d_lines);
	
	std::vector<cv::Vec4i> lines_g;

   	if (! d_lines.empty())
    {
       	lines_g.resize(d_lines.cols);
       	cv::Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_g[0]);
        d_lines.download(h_lines);
   	}

   	for (size_t i = 0; i < lines_g.size(); ++i)
   	{
        cv::Vec4i line_point = lines_g[i];
        cv::line(h_image, cv::Point(line_point[0], line_point[1]), Point(line_point[2], line_point[3]), Scalar(0, 0, 255), 2, LINE_AA);
	}
	
	return lines_g;
}

void parking_mission::checkLeftSonar(const sensor_msgs::Range::ConstPtr& msg)
{
	sonarLeftDist = msg->range;

	if (sonarLeftDist < 0.30)
	{
		lower_obstacle = True;
	}
}

void parking_mission::checkRightSonar(const sensor_msgs::Range::ConstPtr& msg)
{
	sonarRightDist = msg->range;

	if (sonarRightDist < 0.30)
	{
		lower_obstacle = True;
	}
}

void parking_mission:parkingStarts(parkState state)
{
	if (state == reverseCurve)
	{
		turtlemove(-0.11,0.7);
		ros::Time startTimeCount = ros::Time::now();
	}
	
	else if (state = reverseStraight)
	{
		ros::Time endTimeCount = ros::Time::now();
		turtlemove(-0.1,0);
		ros::Duration::sleep(ros::Duration::Duration(1.7));
		turtlemove(0,0);
		turtlemove(0.1,0);
		ros::Duration::sleep(ros::Duration::Duration(1.7));
		turtlemove(0.11,-0.7);
		ros::Duration::sleep(endTimeCount-startTimeCount);
		turtlemove(0,0);	
	}
}
	
void parking_mission::turtlemove(double linear, double angular)
{
	geometry_msgs::Twist command;
	command.linear.x = linear;
	command.angular.z = angular; 
	turtlemove_pub.publish(command); 
}

void parking_mission::checkLidarDist(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	for (int i=90;i<136,i++)
	{
		if ( msg->Ranges[i] > 0.12 && msg->Ranges[i] < 0.30)
		{
			higher_obstacle = true;
		}
	}
}	 			
			
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ParkingMission_Node");
	
	ros::Subscriber signSub = n.subscribe('/sign_detected/parking', 1, parkMissionStarts);
	ros::spin();
	
	while (ros::ok())
	{
		//do nothing
	}
}