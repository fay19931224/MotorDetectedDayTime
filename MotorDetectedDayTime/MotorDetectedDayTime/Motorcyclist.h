#pragma once
#include "ObjManager.h"
#include "DetectedObject.h"
#include "ClassifierType.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class Motorcyclist :public DetectedObject
{
public:
	Motorcyclist(Mat frame,Rect motorcyclistRect, Rect headRect,Scalar motorcyclistColor,Scalar headColor);
	Motorcyclist(Mat frame, Rect motorcyclistRect, HeadDetectReturnStruct* headStrcut, Scalar motorcyclistColor, Scalar headColor);
	~Motorcyclist();
	void UpdateObj(Mat &frame);
	string predictDirect();
	void DrawObj(Mat &frame);
	void DrawObjHead(Mat &frame);
	TrackingObject * GetObject(string type);
	void SetObjectCount(int i, string type);
	void setHeadStruct(cv::Point center,int radius);
private:
	TrackingObject *motorcyclist= NULL;
	TrackingObject *head=NULL;
	HeadDetectReturnStruct *headStruct;
};
