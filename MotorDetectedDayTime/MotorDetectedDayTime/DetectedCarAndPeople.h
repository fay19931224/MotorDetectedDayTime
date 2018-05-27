#pragma once
#include "ObjManager.h"
#include "DetectedObject.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class DetectedCarAndPeople :public DetectedObject
{
public:
	DetectedCarAndPeople(Mat frame, Rect carRect, Scalar carColor);
	~DetectedCarAndPeople();
	void UpdateObj(Mat &frame);
	string predictDirect();
	void DrawObj(Mat &frame);
	TrackingObject * GetObject(string type);
	void SetObjectCount(int i, string type);

private:
	TrackingObject *car;
};