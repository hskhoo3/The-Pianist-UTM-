#include "ros/ros.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class breakingBarMission()
{
	int iLowH
	int iHighH;

	int iLowS; 
	int iHighS;

	int iLowV;
	int iHighV;

	cv::Mat img, imgHSV, imgThresholded, imgEdge;	

	std::vector<cv::Vec4i> lines; 

	int detectRedRect;
	cv::Point2f computeIntersect;
	bool comparator;
	void sortCorners;
};

breakingBarMission::breakingBarMission(void)
{
	ros::Subscriber sub = n.subscribe("/camera/image/compressed_fisheye", 1, detectRedRect);

	iLowH = 200;
	iHighH = 255;

	iLowS = 0; 
	iHighS = 50;

	iLowV = 0;
	iHighV = 50;
}

int breakingBarMission::detectRedRect(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	img = cv::imread(cv_ptr); 

	cv::cvtColor(img, imgHSV, cv::COLOR_BGR2RGB); //Convert the captured frame from BGR to HSV

	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

	cv::imshow("Thresholded Image", imgThresholded);

	cv::Canny(imgThresholded, imgEdge, 50, 200, 3);
  
	cv::HoughLinesP(imgEdge, lines, 1, CV_PI/180, 4, 3, 30);

	std::cout<<"lines : " << lines.size() << std::endl;

	cv::imshow("hough", img2);


	for(int i=0;i<lines.size();i++)
	{
		cv::Point2f point1 (lines[i][0], lines[i][1]);
		cv::Point2f point2 (lines[i][2], lines[i][3]);
		cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 3, CV_AA);
	}

	cv::imshow("drawlines", img);

	int* poly = new int[lines.size()];
  
  for(int i=0;i<lines.size();i++)poly[i] = - 1;  
  int curPoly = 0;  
       std::vector<std::vector<cv::Point2f> > corners;  
      for (int i = 0; i < lines.size(); i++)  
      {  
           for (int j = i+1; j < lines.size(); j++)  
           {  
          
                cv::Point2f pt = computeIntersect(lines[i], lines[j]);  
                if (pt.x >= 0 && pt.y >= 0&&pt.x<img.size().width&&pt.y<img.size().height){ 

                     if(poly[i]==-1&&poly[j] == -1){  
                          std::vector<cv::Point2f> v;  
                          v.push_back(pt);  
                          corners.push_back(v);       
                          poly[i] = curPoly;  
                          poly[j] = curPoly;  
                          curPoly++;  
                          continue;  
                     }  
                     if(poly[i]==-1&&poly[j]>=0){  
                          corners[poly[j]].push_back(pt);  
                          poly[i] = poly[j];  
                          continue;  
                     }  
                     if(poly[i]>=0&&poly[j]==-1){  
                          corners[poly[i]].push_back(pt);  
                          poly[j] = poly[i];  
                          continue;  
                     }  
                     if(poly[i]>=0&&poly[j]>=0){  
                          if(poly[i]==poly[j]){  
                               corners[poly[i]].push_back(pt);  
                               continue;  
                          }  
                        
                          for(int k=0;k<corners[poly[j]].size();k++){  
                               corners[poly[i]].push_back(corners[poly[j]][k]);  
                          }  
                       
                          corners[poly[j]].clear();  
                          poly[j] = poly[i];  
                          continue;  
                     }  
                }  
           }  
      }  

for(int i=0;i<corners.size();i++){  
           cv::Point2f center(0,0);  
           if(corners[i].size()<4)continue;  
           for(int j=0;j<corners[i].size();j++){  
                center += corners[i][j];  
           }  
           center *= (1. / corners[i].size());  
           sortCorners(corners[i], center);  
      }  

for(int i=0;i<corners.size();i++){  
           if(corners[i].size()<4)continue;  
           cv::Rect r = cv::boundingRect(corners[i]);  
           if(r.area()<1)
		{
		continue;  
		}
           std::cout<<r.area()<<std::endl;  

	   cv::rectangle(img, corners[i][0], corners[i][2], cv::Scalar(0, 0, 100));

           std::stringstream ss;  
           ss<<i<<"rect.jpg";  
           cv::imshow(ss.str(), img);
	   cv::waitKey(0); 
      }  

	
}

cv::Point2f breakingBarMission::computeIntersect(cv::Vec4i a, cv::Vec4i b)  
 {  
   int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];  
   int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];  
   if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))  
   {  
     cv::Point2f pt;  
     pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;  
     pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d; 

           if(pt.x<std::min(x1,x2)-5||pt.x>std::max(x1,x2)+5||pt.y<std::min(y1,y2)-5||pt.y>std::max(y1,y2)+5){  
                return cv::Point2f(-1,-1);  
           }  
           if(pt.x<std::min(x3,x4)-5||pt.x>std::max(x3,x4)+5||pt.y<std::min(y3,y4)-5||pt.y>std::max(y3,y4)+5){  
                return cv::Point2f(-1,-1);  
           } 

     return pt;  
   }  
   else  
     return cv::Point2f(-1, -1);  
 }  

bool breakingBarMission::comparator(cv::Point2f a, cv::Point2f b){  
           return a.x<b.x;  
      }  

void breakingBarMission::sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center)  
 {  
   std::vector<cv::Point2f> top, bot;  
   for (int i = 0; i < corners.size(); i++)  
   {  
     if (corners[i].y < center.y)  
       top.push_back(corners[i]);  
     else  
       bot.push_back(corners[i]);  
   }  
      sort(top.begin(),top.end(),comparator);  
      sort(bot.begin(),bot.end(),comparator);  
   cv::Point2f tl = top[0];
   cv::Point2f tr = top[top.size()-1];
   cv::Point2f bl = bot[0];
   cv::Point2f br = bot[bot.size()-1];  
   corners.clear();  
   corners.push_back(tl);  
   corners.push_back(tr);  
   corners.push_back(br);  
   corners.push_back(bl);  
 }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "breakingBarMission_Node");
	ros::NodeHandle n;

	breakingBarMission barDetector;
	
	geometry_msgs::Twist command;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Subscriber sub = n.subscribe("/camera/image/compressed_fisheye", 1, barDetector.detectRedRect);
	ros::spin();

	int count;

	while (ros::ok())
	{
		count = barDetector.detectRedRect();

		if (count>1)
		{
			command.linear.x = 0.0;
			command.linear.x = 0.0;
			command.linear.x = 0.0;

			command.angular.x = 0.0;
			command.angular.y = 0.0;
			command.angular.z = 0.0;

			pub.publish(command);
		}
	}

	return 0;
} 

