#include "DetectedCarAndPeople.h"

DetectedPeople::DetectedPeople(Mat frame, Rect pedRect, Scalar pedColor)
{
	ped = new TrackingObject(frame, pedRect, pedColor);
}

DetectedPeople::~DetectedPeople()
{
	delete ped;
}

void DetectedPeople::UpdateObj(Mat & frame)
{
	ped->ObjUpdate(frame);
}

void DetectedPeople::DrawObj(Mat & frame)
{
	ped->DrawObj(frame);
}

TrackingObject * DetectedPeople::GetObject(string type)
{
	return ped;
}

