#pragma once
#include <iostream>
#include "ClassifierType.h"
#include "PrimalSVM.h"
#include <opencv2\core.hpp>
#include <opencv2\ml\ml.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using cv::Mat;
using cv::HOGDescriptor;
using cv::Size;
using cv::Ptr;
using cv::Rect;


using namespace cv::ml;
using namespace std;

class HeadDetecter
{
public:	
	HeadDetecter();
	HeadDetecter(string featureName, HogParameter hogParameter);
	~HeadDetecter();
	bool detectedHeadHoughCircles(Mat grayFrame, Rect roi, HeadDetectStruct* headDetectStruct);	
	bool detectedHeadHOGSVM(Mat grayFrame, Rect roi, HeadDetectStruct* headDetectStruct);
	bool detectedHead(Mat grayFrame, Rect roi, HeadDetectStruct* headDetectStruct);
private:	
	HOGDescriptor* _hogDescriptor=NULL;	
	HeadDetectType _detectType;
};

