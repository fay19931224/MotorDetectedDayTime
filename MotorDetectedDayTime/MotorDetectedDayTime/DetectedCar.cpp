#include "DetectedCar.h"

DetectedCar::DetectedCar(Mat frame, Rect carRect, Scalar carColor)
{
	car = new TrackingObject(frame, carRect, 0, carColor);
}

DetectedCar::~DetectedCar()
{

}

void DetectedCar::UpdateObj(Mat & frame)
{
	car->ObjUpdate(frame);
}

string DetectedCar::predictDirect()
{
	return "";
}

void DetectedCar::DrawObj(Mat & frame)
{
	car->DrawObj(frame);
}

TrackingObject * DetectedCar::GetObject(string type)
{
	return car;
}

void DetectedCar::SetObjectCount(int i, string type)
{
	car->detectionCount += i;
}
