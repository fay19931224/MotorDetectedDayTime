#include "HeadDetecter.h"

HeadDetecter::HeadDetecter() 
{

}

HeadDetecter::~HeadDetecter()
{
}

bool HeadDetecter::detectedHeadHoughCircles(Mat grayFrame, Rect roi, HeadDetectReturnStruct* headDetectReturnStruct)
{		
	if (roi.x + roi.width > grayFrame.cols) 
	{
		roi.width = grayFrame.cols - roi.x;
	}
	else if(roi.x<0)
	{
		roi.x = 0;		
	}
	Mat sample = grayFrame(roi);
	vector<cv::Vec3f> circles;
	double dp = 1;
	double minDist = sample.rows;	//圓彼此間的最短距離
	if (sample.rows < sample.cols)
	{
		minDist = sample.cols;
	}
	double param1 = 100;			//呼叫Canny()尋找邊界，param1就是Canny()的高閾值，低閾值自動設為此值的一半，大於高閥值才會被視為邊緣。	
	double param2 = 15;				//計數閾值，超過此值的圓才會存入。
	int minRadius = sample.cols / 5;
	int maxRadius = sample.cols / 3;
	if (sample.cols > sample.rows)
	{
		minRadius = sample.rows / 5;
		maxRadius = sample.rows / 3;
	}
	HoughCircles(sample, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
	if (circles.size() != 0)
	{
		cv::Point center(cvRound(circles[0][0])+roi.x, cvRound(circles[0][1])+roi.y);
		int radius = cvRound(circles[0][2]);		
		headDetectReturnStruct->center = center;
		headDetectReturnStruct->radius = radius;			
		return true;
	}

	Mat sample2 = grayFrame(roi);
	Mat dst;
	Sobel(sample2, dst, CV_8U, 1, 1);
	Mat abs_dst;
	convertScaleAbs(dst, abs_dst);
	Mat add_dst;
	addWeighted(sample2, 1, abs_dst, -1, CV_8U, add_dst);
	double dp2 = 1;
	double minDist2 = sample2.rows;
	if (sample2.rows < sample2.cols)
	{
		minDist2 = sample2.cols;
	}
	double param12 = 50;
	double param22 = 5;
	int minRadius2 = sample.cols / 5;
	int maxRadius2 = sample.cols / 3;
	if (sample.cols > sample.rows)
	{
		minRadius2 = sample.rows / 5;
		maxRadius2 = sample.rows / 3;
	}
	vector<cv::Vec3f> circles2;
	HoughCircles(add_dst, circles2, CV_HOUGH_GRADIENT, dp2, minDist2, param12, param22, minRadius2, maxRadius2);

	if (circles2.size() != 0)
	{
		cv::Point center(cvRound(circles2[0][0]) + roi.x, cvRound(circles2[0][1]) + roi.y);
		int radius = cvRound(circles2[0][2]);
		headDetectReturnStruct->center = center;
		headDetectReturnStruct->radius = radius;
		return true;
	}
	return false;
}