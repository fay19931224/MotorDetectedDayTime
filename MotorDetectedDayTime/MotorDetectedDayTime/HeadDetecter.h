#pragma once
#include <iostream>
#include "ClassifierType.h"
#include <opencv2\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using cv::Mat;
using cv::Size;
using cv::Rect;

using namespace std;
class HeadDetecter
{
public:	
	HeadDetecter();
	bool detectedHeadHoughCircles(Mat grayFrame, Rect roi, HeadDetectReturnStruct* headDetectReturnStruct);	
	~HeadDetecter();
private:
	
};

