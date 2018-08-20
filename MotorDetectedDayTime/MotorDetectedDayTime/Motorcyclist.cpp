#include "Motorcyclist.h"

Motorcyclist::Motorcyclist(Mat frame, Rect motorcyclistRect, HeadDetectStruct* headStrcut, Scalar motorcyclistColor, Scalar headColor)
{
	motorcyclist = new TrackingObject(frame, motorcyclistRect, motorcyclistColor);
	headStruct = headStrcut;
}

Motorcyclist::~Motorcyclist()
{
	delete motorcyclist;
}

void Motorcyclist::UpdateObj(Mat &frame)
{		
	motorcyclist->ObjUpdate(frame);	
}

void Motorcyclist::DrawObj(Mat &frame)
{
	motorcyclist->DrawObj(frame);
}

void Motorcyclist::DrawObjHead(Mat & frame)
{
	if (headStruct->detectType == HeadDetectType::HOUGH) 
	{
		circle(frame, headStruct->center, headStruct->radius, Scalar(255, 255, 255), 2, 8, 0);
	}
	else if(headStruct->detectType == HeadDetectType::HOGSVM)
	{
		rectangle(frame, headStruct->detectedRect, Scalar(255, 255, 255), 2);
	}
	
}



TrackingObject*  Motorcyclist::GetObject()
{
	return motorcyclist;
}


