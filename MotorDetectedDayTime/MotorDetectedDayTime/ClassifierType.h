#ifndef CLASSIFIER_TYPE_H
#define CLASSIFIER_TYPE_H


using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
/*!
* 此.h檔提供了使用的分類型態，以及與LIDAR結合時該使用哪組對應參數
*/
enum ClassiferType
{	
	MotorbikeFrontBack,
	MotorbikeSide,
	Helmet
};

enum FusionType
{
	CarFront,
	CarRightSide,
	CarLeftSide,
	CarBack
};

typedef struct svmDetectParameter 
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
#endif;