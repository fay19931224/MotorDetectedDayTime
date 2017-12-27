#ifndef CLASSIFIER_TYPE_H
#define CLASSIFIER_TYPE_H

#include <opencv2\core.hpp>
#include <opencv2\ml\ml.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include  <string>
#include <ctime>
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
/*!
* ��.h�ɴ��ѤF�ϥΪ��������A�A�H�λPLIDAR���X�ɸӨϥέ��չ����Ѽ�
*/
enum ClassiferType
{
	MotorbikeFrontBack,
	MotorbikeSide,	
	CarFrontBack,
	Pedestrian,
	Helmet
};

enum FusionType
{
	CarFront,
	CarRightSide,
	CarLeftSide,
	CarBack
};

struct svmDetectParameter 
{	
	Size WINDOW_SIZE;
	Size CELL_SIZE;	
	float hitThreshold;
	Size winStride;
	Size padding;
	double scale;
	double finalThreshold;
	bool useMeanshiftGrouping;
};

struct HeadSVMDetectReturnStruct
{
	bool isError;
	bool isDetected;
	cv::Rect detectedRect;
	float scale;
};

struct SentData 
{
	int ROI_Left_Top_X;
	int ROI_Left_Top_Y;
	int ROI_Width;
	int ROI_Height;
	double Object_Distance;
	std::string Object_Type;
};
#endif;