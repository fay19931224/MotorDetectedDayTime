#pragma once

#include "kcftracker.hpp"
//#include "FusionManager.h"
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
using namespace std;
/*!
*��class�O�ΨӶi�檫�󪺰l��
*/
class TrackingObject
{
public:
	TrackingObject(cv::Mat &frame, cv::Rect target, Scalar color);
	~TrackingObject();
	void ObjUpdate(cv::Mat &frame);
	void DrawObj(cv::Mat &frame);
	Rect getROI();

	float confidence();
	Rect TrackingRect;	
	Scalar _color;	
	bool isChecked = false;	
	int missCount = 0;	
	bool isDetected=false;	
	KCFTracker *tracker;
private:	
};

