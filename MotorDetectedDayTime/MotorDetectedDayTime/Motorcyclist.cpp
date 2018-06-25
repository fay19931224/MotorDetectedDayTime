#include "Motorcyclist.h"

Motorcyclist::Motorcyclist(Mat frame, Rect motorcyclistRect, HeadDetectReturnStruct* headStrcut, Scalar motorcyclistColor, Scalar headColor)
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
	circle(frame, headStruct->center, headStruct->radius, Scalar(255, 255, 255), 2, 8, 0);
}



TrackingObject*  Motorcyclist::GetObject(string type)
{
	string type1 = "moto";	
	if (type.compare(type1) == 0)
	{		
		return motorcyclist;
	}	
	return new TrackingObject(Mat(), Rect(), Scalar());
}

void Motorcyclist::setHeadStruct(cv::Point center, int radius)
{
	headStruct->center = center;
	headStruct->radius = radius;
}







