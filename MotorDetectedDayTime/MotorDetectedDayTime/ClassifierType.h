#ifndef CLASSIFIER_TYPE_H
#define CLASSIFIER_TYPE_H

#include <opencv2\core.hpp>
#include <opencv2\highgui\highgui.hpp>
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;

enum ClassiferType
{
	MotorbikeFrontBack,
	MotorbikeSide,		
	Pedestrian,
	CarFrontBack,
	Helmet
};

enum FusionType
{
	CarFront,
	CarRightSide,
	CarLeftSide,
	CarBack
};

enum HeadDetectType 
{
	HOGSVM,
	HOUGH
};

struct HogParameter 
{	
	Size winSize;
	Size cellSize;	
	float hitThreshold;
	Size winStride;
	Size padding;
	double scale;
	double finalThreshold;
	bool useMeanshiftGrouping;
	int normalizeType;
	float horion;
	bool version;
};

struct HeadDetectStruct
{	
	HeadDetectType detectType;
	bool isDetected;
	cv::Point center;
	int radius;
	cv::Rect detectedRect;	
};

#endif;