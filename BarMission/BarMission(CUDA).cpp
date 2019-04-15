#include "ros/ros.h"
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/gpu/gpu.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/CompressedImage.h"

ros::NodeHandle n;
bool comparator(cv::Point2f a, cv::Point2f b);

bool comparator(cv::Point2f a, cv::Point2f b)
{  
	return a.x<b.x;  
}

class BarMission()
{
	public:
	cv_bridge::CvImagePtr cv_ptr;

	cv::Mat h_image, h_edge;
	cv::cuda::GpuMat d_edge, d_image, d_frame_hsv, d_intermediate,d_result, d_imagec;
	cv::cuda::GpuMat d_frame_shsv[3];
	cv::cuda::GpuMat d_thresc[3];

	cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge;
	Ptr<cuda::HoughSegmentDetector> hough;

	std::vector<std::vector<cv::Point2f>> corners; 

	void detectRedRect(const sensor_msgs::Image::ConstPtr& msg);
	cv::Point2f computeIntersect(cv::Vec4i a, cv::Vec4i b);
	void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center);
};

BarMission::BarMission(void)
{
	ros::Subscriber sub = n.subscribe("/camera/image/compressed_fisheye", 1, &BarMission::detectRedRect, this);	

	canny_edge = cv::cuda::createCannyEdgeDetector(2.0, 100.0, 3, false);
	hough = cuda::createHoughSegmentDetector(1.0f, (float) (CV_PI / 180.0f), 50, 5);
	
}

void BarMission::detectRedRect(const sensor_msgs::Image::ConstPtr& msg)
{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

	d_image.upload(cv_ptr->image);

	cv::cuda::cvtColor(d_frame, d_frame_hsv, COLOR_BGR2HSV);

	//Split HSV 3 channels
	cv::cuda::split(d_frame_hsv, d_frame_shsv);

	//Threshold HSV channels
	cv::cuda::threshold(d_frame_shsv[0], d_thresc[0], 110, 130, THRESH_BINARY);
	cv::cuda::threshold(d_frame_shsv[1], d_thresc[1], 50, 255, THRESH_BINARY);
	cv::cuda::threshold(d_frame_shsv[2], d_thresc[2], 50, 255, THRESH_BINARY);

	//Bitwise AND the channels
	cv::cuda::bitwise_and(d_thresc[0], d_thresc[1],d_intermediate);
	cv::cuda::bitwise_and(d_intermediate, d_thresc[2], d_result);

	canny_edge->detect(d_result, d_edge);

	hough->detect(d_edge, d_lines);

	std::vector<cv::Vec4i> lines_g;

   	if (!d_lines.empty())
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
	
	int* poly = new int [lines_g.size()];  

	for(int i=0;i<lines_g.size();i++)
		poly[i] = - 1;  


	int curPoly = 0;  

	for (int i = 0; i < lines_g.size(); i++)  
      	{  
        	for (int j = i+1; j < lines_g.size(); j++)  
           	{  
                	cv::Point2f pt = computeIntersect(lines_g[i], lines_g[j]);
  
                	if (pt.x >= 0 && pt.y >= 0 && pt.x<cv_ptr.size().width && pt.y<cv_ptr.size().height)
			{  
                     		if(poly[i]== -1 && poly[j] == -1)
				{  
		                  	std::vector<cv::Point2f> v;  
		                  	v.push_back(pt);  
		                  	corners.push_back(v);       
		                  	poly[i] = curPoly;  
		                  	poly[j] = curPoly;  
		                  	curPoly++;  
		                  	continue;  
                     		}  

                     		if(poly[i]==-1 && poly[j]>=0)
				{  
                          		corners[poly[j]].push_back(pt);  
                          		poly[i] = poly[j];  
                          		continue;  
                 		}  

                     		if(poly[i]>=0 && poly[j]==-1)
				{  
                          		corners[poly[i]].push_back(pt);  
                          		poly[j] = poly[i];  
                          		continue;  
                     		}
  
                     		if(poly[i]>=0 && poly[j]>=0)
				{  
                          		if(poly[i]==poly[j])
					{  
                               			corners[poly[i]].push_back(pt);  
                               			continue;  
                          		}  
                        
                          		for(int k=0;k<corners[poly[j]].size();k++)
					{  
                               			corners[poly[i]].push_back(corners[poly[j]][k]);  
                          		}  
                       
                          		corners[poly[j]].clear();  
                          		poly[j] = poly[i];  
                          		continue;  
                     		}  
                	}  
           	}  
      	}

	for(int i=0;i<corners.size();i++)
	{  
        	cv::Point2f center(0,0);  

           	if(corners[i].size()<4)
			continue;

           	for(int j=0;j<corners[i].size();j++)
		{  
                	center += corners[i][j];  
           	}
  
           center *= (1. / corners[i].size());  
           sortCorners(corners[i], center);  
      	}

	int rectCount = 0;

	for (int i=0;i<corners.size();i++)
	{  
           if (corners[i].size()<4)
		continue;  

           cv::Rect r = boundingRect(corners[i]); 
 
           if (r.area()<10)
		continue;  

	   rectCount++;
           cout<<r.area()<<endl;  
 
           //stringstream ss;  
          // ss<<i<<".jpg";  
           //imshow(ss.str(), quad);  
      	}    
}

cv::Point2f BarMission::computeIntersect(cv::Vec4i a, cv::Vec4i b)  
{  
	int x1 = a[0], y1 = a[1], x2 = a[2], y2 = a[3];  
   	int x3 = b[0], y3 = b[1], x4 = b[2], y4 = b[3];  

   	if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))  
   	{  
     		cv::Point2f pt;  
     		pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;  
     		pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;  
           	//-10 is a threshold, the POI can be off by at most 10 pixels
           	if (pt.x<min(x1,x2)-10||pt.x>max(x1,x2)+10||pt.y<min(y1,y2)-10||pt.y>max(y1,y2)+10)
		{  
                	return cv::Point2f(-1,-1);  
           	} 
 
           	if (pt.x<min(x3,x4)-10||pt.x>max(x3,x4)+10||pt.y<min(y3,y4)-10||pt.y>max(y3,y4)+10)
		{  
                	return cv::Point2f(-1,-1);  
           	}  

     		return pt;  
   	}  

   	else  
     		return cv::Point2f(-1, -1);  
}
  
void BarMission::sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center)  
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
	ros::init(argc, argv, "BarMissionNode");

	BarMission controller;
	
	while (ros::ok())
	{

	}
}
