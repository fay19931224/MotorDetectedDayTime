#pragma once

#include "kcftracker.hpp"
//#include "FusionManager.h"
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
using namespace std;
/*!
*此class是用來進行物件的追蹤
*/
class TrackingObject
{
public:
	TrackingObject(cv::Mat &frame, cv::Rect target, int id, Scalar color);
	~TrackingObject();
	void ObjUpdate(cv::Mat &frame);
	void DrawObj(cv::Mat &frame);
	Rect getROI();
	void updateROI(cv::Rect newROI);
	float confidence();
	cv::Rect TrackingRect;	
	Scalar _color;
	int _id = 0;
	bool isNewDetection = true;
	int detectionCount = 0;
	int missCount = 0;
	bool isTracking;	
	bool isDetected=false;
	Rect initRect;
private:
	KCFTracker *tracker;
	int name = 0;
};

