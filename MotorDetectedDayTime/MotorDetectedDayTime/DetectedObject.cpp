#include "DetectedObject.h"
DetectedObject::DetectedObject()
{
}

DetectedObject::~DetectedObject()
{
}

void DetectedObject::UpdateObj(Mat & frame)
{
}

string DetectedObject::predictDirect()
{
	return "";
}

void DetectedObject::DrawObj(Mat & frame, bool isHelmetdraw)
{
}

TrackingObject * DetectedObject::GetObject(string type)
{
	return nullptr;
}
