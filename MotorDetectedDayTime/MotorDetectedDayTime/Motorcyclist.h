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
	Motorcyclist(Mat frame, Rect motorcyclistRect, HeadDetectStruct* headStrcut, Scalar motorcyclistColor, Scalar headColor);
	~Motorcyclist();
	void UpdateObj(Mat &frame);	
	void DrawObj(Mat &frame);
	void DrawObjHead(Mat &frame);
	TrackingObject * GetObject();	
private:
	TrackingObject *motorcyclist= NULL;	
	HeadDetectStruct *headStruct;
};
