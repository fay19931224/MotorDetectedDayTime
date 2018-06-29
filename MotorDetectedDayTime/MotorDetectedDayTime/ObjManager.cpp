#include "ObjManager.h"


TrackingObject::TrackingObject(cv::Mat &frame, cv::Rect target, Scalar color)
{
	TrackingRect = target;	
	_color = color;	
	tracker = new KCFTracker(true, false, true, true);
	tracker->init(target, frame);		
}

TrackingObject::~TrackingObject()
{	
	delete tracker;
}

void TrackingObject::ObjUpdate(cv::Mat &frame)
{
	TrackingRect = tracker->update(frame);
}

void TrackingObject::DrawObj(cv::Mat &frame)
{	
	cv::rectangle(frame, TrackingRect, _color, 2);	
}

Rect TrackingObject::getROI()
{
	return TrackingRect;
}

float TrackingObject::confidence()
{
	return tracker->peak_value;
}