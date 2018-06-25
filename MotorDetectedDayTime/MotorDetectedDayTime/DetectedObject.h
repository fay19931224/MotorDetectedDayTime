#pragma once
#include "ObjManager.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class DetectedObject
{
public:
	DetectedObject();
	~DetectedObject();
	virtual void UpdateObj(Mat &frame);
	virtual string predictDirect();
	virtual void DrawObj(Mat &frame);
	virtual TrackingObject * GetObject(string type);
	bool isduplicate = false;
private:
	
};

