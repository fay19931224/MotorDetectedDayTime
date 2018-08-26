#include "HeadDetecter.h"

HeadDetecter::HeadDetecter() 
{
	_detectType = HeadDetectType::HOUGH;
}

HeadDetecter::HeadDetecter(string featureName, HogParameter hogParameter)
{
	PrimalSVM* svm = new PrimalSVM(featureName);
	vector<float> hogVector;
	svm->getSupportVector(hogVector);
	Size _winSize = hogParameter.winSize;
	Size _blockSize = Size(hogParameter.cellSize.width * 2, hogParameter.cellSize.height * 2);
	Size _blockStride = hogParameter.winStride;
	Size _cellSize = hogParameter.cellSize;
	int _normalizeType = hogParameter.normalizeType;
	int _nbins = 9;
	_hogDescriptor = new HOGDescriptor(hogParameter.winSize, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS, false);
	_hogDescriptor->setSVMDetector(hogVector);
	_detectType = HeadDetectType::HOGSVM;
}

HeadDetecter::~HeadDetecter()
{

}

bool HeadDetecter::detectedHeadHoughCircles(Mat grayFrame, Rect roi, HeadDetectStruct* headDetectStruct)
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
	double dp = 1.0;
	double minDist = sample.rows;	
	if (sample.rows < sample.cols)
	{
		minDist = sample.cols;
	}
	double param1 = 100;
	double param2 = 15;
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
		headDetectStruct->detectType= HeadDetectType::HOUGH;
		headDetectStruct->center = center;
		headDetectStruct->radius = radius;
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
		headDetectStruct->detectType = HeadDetectType::HOUGH;
		headDetectStruct->center = center;
		headDetectStruct->radius = radius;
		return true;
	}
	return false;
}

bool HeadDetecter::detectedHeadHOGSVM(Mat grayFrame, Rect roi, HeadDetectStruct* headDetectStruct)
{
	if (roi.x + roi.width > grayFrame.cols)
	{
		roi.width = grayFrame.cols - roi.x;
	}
	else if (roi.x<0)
	{
		roi.x = 0;
	}
	if (roi.width<_hogDescriptor->winSize.width || roi.height<_hogDescriptor->winSize.height)
	{
		return false;
	}
	vector<Rect> result;
	vector<double> resultWeight;

	//_hogDescriptor->detectMultiScale(grayFrame(roi), result, resultWeight, 0.0, Size(2, 2), Size(), 1.01, 2.0, false);
	_hogDescriptor->detectMultiScale(grayFrame(roi), result, resultWeight, 0.0, Size(2, 2), Size(), 1.05, 2.0, false);
	int Max = INT32_MIN;
	int maxIndex = -1;
	for (int i = 0; i < resultWeight.size(); i++)
	{
		if (resultWeight[i] > Max)
		{
			Max = resultWeight[i];
			maxIndex = i;
		}
	}	
	if (maxIndex != -1)
	{		
		headDetectStruct->detectType = HeadDetectType::HOGSVM;
		headDetectStruct->detectedRect = Rect(result[maxIndex].x+roi.x, result[maxIndex].y + roi.y, result[maxIndex].width, result[maxIndex].height);
		return true;
	}	
	return false;
}

bool HeadDetecter::detectedHead(Mat grayFrame, Rect roi, HeadDetectStruct * headDetectStruct)
{
	if (_detectType == HeadDetectType::HOUGH)
	{
		return detectedHeadHoughCircles(grayFrame, roi, headDetectStruct);
	}
	else if (_detectType == HeadDetectType::HOGSVM)
	{
		return detectedHeadHOGSVM(grayFrame, roi, headDetectStruct);
	}
	std::cout << "error" << std::endl;
	return false;
}
