#include "Motorcyclist.h"

Motorcyclist::Motorcyclist(Mat frame, Rect motorcyclistRect, Rect headRect, Scalar motorcyclistColor, Scalar headColor)
{
	motorcyclist = new TrackingObject(frame, motorcyclistRect, 0, motorcyclistColor);
	head = new TrackingObject(frame, headRect, 0, headColor);	
}

Motorcyclist::~Motorcyclist()
{

}

void Motorcyclist::UpdateObj(Mat &frame)
{	
	motorcyclist->ObjUpdate(frame);
	head->ObjUpdate(frame);
}

string Motorcyclist::predictDirect()
{
	int motorcyclistPos = motorcyclist->getROI().x + motorcyclist->getROI().width / 2;
	int headlistPos = head->getROI().x + head->getROI().width / 2;
	if (abs(headlistPos - motorcyclistPos)<10) 
	{
		return string("");
	}
	else if (headlistPos > motorcyclistPos) 
	{
		return "right";
	}
	else {
		return "left";
	}
	
}

void Motorcyclist::DrawObj(Mat &frame)
{
	motorcyclist->DrawObj(frame);
}

void Motorcyclist::DrawObjHead(Mat & frame)
{
	head->DrawObj(frame);
}

TrackingObject*  Motorcyclist::GetObject(string type)
{
	string type1 = "motorcyclist";
	string type2 = "head";
	if (type.compare(type1) == 0)
	{		
		return motorcyclist;
	}
	else if (type.compare(type2) == 0)
	{
		return head;
	}
	return new TrackingObject(Mat(), Rect(), 0, Scalar());
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





