#pragma once
#include "ObjManager.h"
using cv::Mat;
using cv::Rect;
using cv::Scalar;
class Motorcyclist
{
public:
	Motorcyclist(Mat &frame,Rect motorcyclistRect, Rect headRect,Scalar motorcyclistColor,Scalar headColor);
	~Motorcyclist();
	void Update(Mat &frame);
	void DrawObj(Mat &frame);
	TrackingObject GetObject(string type);
	void SetObjectCount(int i, string type);
	
private:
	TrackingObject *motorcyclist;
	TrackingObject *head;
};


Motorcyclist::Motorcyclist(Mat & frame, Rect motorcyclistRect, Rect headRect, Scalar motorcyclistColor, Scalar headColor)
{
	motorcyclist = new TrackingObject(frame, motorcyclistRect, 0, motorcyclistColor);
	head=new TrackingObject(frame, headRect, 0, headColor);
}

Motorcyclist::~Motorcyclist()
{

}

void Motorcyclist::Update(Mat &frame)
{
	motorcyclist->ObjUpdate(frame);
	head->ObjUpdate(frame);
}

void Motorcyclist::DrawObj(Mat & frame)
{
	motorcyclist->DrawObj(frame, Scalar());
	head->DrawObj(frame, Scalar());
}

TrackingObject Motorcyclist::GetObject(string type)
{
	string type1 = "motorcyclist";
	string type2 = "head";
	if (type.compare(type1) == 0)
	{
		return *motorcyclist;
	}
	else if (type.compare(type2) == 0)
	{
		return *head;
	}	
	return TrackingObject(Mat(), Rect(), 0, Scalar());
}

void Motorcyclist::SetObjectCount(int i, string type)
{
	string type1 = "motorcyclist";
	string type2 = "head";
	if (type.compare(type1) == 0)
	{
		motorcyclist->detectionCount += i;
	}
	else if (type.compare(type2) == 0)
	{
		head->detectionCount += i;
	}	
}





