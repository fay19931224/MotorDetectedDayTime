#pragma once
#include "ObjManager.h"
#include "DetectedObject.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class DetectedCar :public DetectedObject
{
public:
	DetectedCar(Mat frame, Rect carRect, Scalar carColor);
	~DetectedCar();
	void UpdateObj(Mat &frame);
	string predictDirect();
	void DrawObj(Mat &frame);
	TrackingObject * GetObject(string type);
	void SetObjectCount(int i, string type);

private:
	TrackingObject *car;
};