#include "ros/ros.h"
#include <cmath>
#include <math.h> 
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Range.h"

#define PI 3.14159265

ros::Publisher turtlemove_pub;

class parkingMission
{
	public:
	cv::Mat d_edge, d_image, d_frame_hsv, d_intermediate,d_result, d_imagec;
	cv::Mat h_image, h_edge, gray, thr, blur, edge, white_lines;
	
	cv_bridge::CvImagePtr cv_ptr;
	
	std::vector<cv::Vec4i> whiteLines;

	float sonarRightDist;
	bool lower_obstacle;
	bool higher_obstacle;
	bool whiteLineDetect;

	std::vector<cv::Vec4i> lines_g;
		
	enum parkState 
	{
		reverseCurve, reverseStraight
	};

	parkingMission(void);	
	void mainProcessing(const sensor_msgs::ImageConstPtr& msg);
	std::vector<cv::Vec4i> detectLine(cv::Mat colImage);
	void checkRightSonar(sensor_msgs::Range msg);
	void parkingStarts(parkState state);
	void turtlemove(double linear, double angular);
	void checkLidarDist(sensor_msgs::LaserScan msg);
};
		
parkingMission::parkingMission(void)
{
	lower_obstacle = false;
	higher_obstacle = false;
	whiteLineDetect = false;
	
}

void parkingMission::mainProcessing(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	whiteLines = detectLine(cv_ptr->image);
	
	if (! whiteLines.empty())
	{
		for (int i=0;i<whiteLines.size();i++)
		{
			cv::Vec4i line_pointw = whiteLines[i];
			
			double degree = atan2((line_pointw[3]-line_pointw[1]),(line_pointw[2]-line_pointw[0]));
			
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
	
	
	if (whiteLineDetect == true)
	{
		turtlemove(0.06,0.00);
	}
	
	else
	{
		if (higher_obstacle == true || lower_obstacle == true)
		{
			turtlemove(0.06,0.00);
		}
		
		else
		{
			parkState currParkState = reverseCurve;
			parkingStarts(currParkState);
		}	
	}
	
}


std::vector<cv::Vec4i> parkingMission::detectLine(cv::Mat colImage)
{ 
	cv::cvtColor(colImage, gray, 6);
	double imgProperties[2] = {gray.cols, gray.rows};
	cv::Mat ROI = gray(cv::Rect(0,(imgProperties[1]/3),imgProperties[1],(imgProperties[1]/3)));
        cv::GaussianBlur(ROI,ROI,cv::Size(9,9),0);
	cv::adaptiveThreshold(ROI,thr,255,1,0,11,2);
        cv::medianBlur(thr,blur,9);
        cv::Canny(blur,edge,0,360);
        cv::HoughLinesP(edge,white_lines,1,PI/180,30) ;

   	if (! white_lines.empty())
  	{
       		lines_g.resize(white_lines.cols);
       		cv::Mat h_lines(1, white_lines.cols, CV_32SC4, &lines_g[0]);
   	}

   	for (size_t i = 0; i < lines_g.size(); ++i)
   	{
        cv::Vec4i line_point = lines_g[i];
        cv::line(h_image, cv::Point(line_point[0], line_point[1]), cv::Point(line_point[2], line_point[3]), cv::Scalar(0, 0, 255), 2, CV_AA);
	}
	
	return lines_g;
}

void parkingMission::checkRightSonar(sensor_msgs::Range msg)
{
	sonarRightDist = msg.range;

	if (sonarRightDist < 0.30)
	{
		lower_obstacle = true;
	}
}

void parkingMission::parkingStarts(parkState state)
{
	ros::Time startTimeCount, endTimeCount;

	if (state == reverseCurve)
	{
		turtlemove(-0.11,0.7);
		startTimeCount = ros::Time::now();
	}
	
	else if (state = reverseStraight)
	{
		endTimeCount = ros::Time::now();
		turtlemove(-0.1,0);
		ros::Duration(1.5).sleep();
		turtlemove(0,0);
		turtlemove(0.1,0);
		ros::Duration(1.5).sleep();
		turtlemove(0.11,-0.7);
		ros::Duration(endTimeCount-startTimeCount).sleep();;
		turtlemove(0,0);	
	}
}
	
void parkingMission::turtlemove(double linear, double angular)
{
	geometry_msgs::Twist command;
	command.linear.x = linear;
	command.angular.z = angular; 
	turtlemove_pub.publish(command); 
}

void parkingMission::checkLidarDist(sensor_msgs::LaserScan msg)
{
	for (int i=90;i<136;i++)
	{
		if ( msg.ranges[i] > 0.12 && msg.ranges[i] < 0.30)
		{
			higher_obstacle = true;
		}
	}
}	 			
			
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ParkingMissionNode");
	ros::NodeHandle n;
	
	parkingMission parkCtrl;
	ros::Rate loop_rate(5.0);
	
	turtlemove_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

	ros::Subscriber lidarSub = n.subscribe("/scan", 1, &parkingMission::checkLidarDist, &parkCtrl);
	ros::Subscriber RightSonarSub = n.subscribe("sonar/right", 1, &parkingMission::checkRightSonar, &parkCtrl);
	ros::Subscriber imageSub = n.subscribe("camera/image/compressed_fisheye", 1, &parkingMission::mainProcessing, &parkCtrl);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
