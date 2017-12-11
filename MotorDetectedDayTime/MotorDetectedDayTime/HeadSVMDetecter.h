#pragma once
#include <iostream>
#include "ClassifierType.h"
#include <opencv2\core.hpp>
#include <opencv2\ml\ml.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <string>
#include <thread>


#define FEATURESAMOUNT 324
#define POSITIVESAMPLEAMOUNT 1383
#define NEGATIVESAMPLEAMOUNT 1667

using cv::Mat;
using cv::HOGDescriptor;
using cv::Size;
using cv::Ptr;
using cv::Rect;
using cv::noArray;

using namespace cv::ml;
using namespace std;
class HeadSVMDetecter
{
public:
	HeadSVMDetecter(string headSVM_XMLFilePath);
	HeadSVMDetectReturnStruct detectedHead(Mat &grayFrame,Rect roi);
	bool detectedHeadhHOG(Mat &frame, Rect roi);
	void draw();
	~HeadSVMDetecter();
private:
	HeadSVMDetectReturnStruct _headSVMDetectReturnStruct;
	void goDetectedHead(float scale, Mat temp, Mat image, Rect roi, HeadSVMDetectReturnStruct &myReturn);
	HOGDescriptor* _hogDescriptor;		
	Ptr<SVM> svm;
	Size WINDOW_SIZE= Size(32, 32);
};

