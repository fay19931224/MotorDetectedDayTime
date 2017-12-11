#pragma once
#include "ObjManager.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class Motorcyclist
{
public:
	Motorcyclist(Mat frame,Rect motorcyclistRect, Rect headRect,Scalar motorcyclistColor,Scalar headColor);
	~Motorcyclist();
	void UpdateObj(Mat &frame);
	string predictDirect();
	void DrawObj(Mat &frame);
	TrackingObject * GetObject(string type);
	void SetObjectCount(int i, string type);
	
private:
	TrackingObject *motorcyclist;
	TrackingObject *head;
};
