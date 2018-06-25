#pragma once
#include "ObjManager.h"
#include "DetectedObject.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class DetectedPeople :public DetectedObject
{
public:
	DetectedPeople(Mat frame, Rect carRect, Scalar carColor);
	~DetectedPeople();
	void UpdateObj(Mat &frame);
	void DrawObj(Mat &frame);
	TrackingObject * GetObject(string type);	
private:
	TrackingObject *ped=nullptr;
};