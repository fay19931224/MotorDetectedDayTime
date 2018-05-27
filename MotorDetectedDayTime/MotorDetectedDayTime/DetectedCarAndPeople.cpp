#include "DetectedCarAndPeople.h"

DetectedCarAndPeople::DetectedCarAndPeople(Mat frame, Rect carRect, Scalar carColor)
{
	car = new TrackingObject(frame, carRect, 0, carColor);
}

DetectedCarAndPeople::~DetectedCarAndPeople()
{

}

void DetectedCarAndPeople::UpdateObj(Mat & frame)
{
	car->ObjUpdate(frame);
}

string DetectedCarAndPeople::predictDirect()
{
	return "";
}

void DetectedCarAndPeople::DrawObj(Mat & frame)
{
	car->DrawObj(frame);
}

TrackingObject * DetectedCarAndPeople::GetObject(string type)
{
	return car;
}

void DetectedCarAndPeople::SetObjectCount(int i, string type)
{
	car->detectionCount += i;
}
