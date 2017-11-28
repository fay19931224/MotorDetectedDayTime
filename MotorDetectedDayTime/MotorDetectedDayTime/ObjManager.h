#pragma once

#include "kcftracker.hpp"
//#include "FusionManager.h"
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
using namespace std;
/*!
*此class是用來進行物件的追蹤
*/
class TrackingObject
{
public:
	TrackingObject(cv::Mat &frame, cv::Rect target, int id, Scalar color);
	~TrackingObject();
	void ObjUpdate(cv::Mat &frame);
	void DrawObj(cv::Mat &frame, cv::Scalar& color);
	Rect getROI();
	void updateROI(cv::Rect newROI);
	float confidence();
	cv::Rect TrackingRect;	
	Scalar _color;
	int _id = 0;
	bool isNewDetection = true;
	int detectionCount = 0;
	int missCount = 0;
	bool isTracking;
private:
	KCFTracker *tracker;
	int name = 0;
};

/*!
*此class是用來進行更新Tracker並顯示於畫面
*/
class ObjManager
{
public:
	ObjManager();
	void update(cv::Mat &frame, std::vector<cv::Rect> &obj, cv::Scalar& color);	
	void draw(cv::Mat& frame, cv::Scalar& color);	
	~ObjManager();
	vector<Rect> _restTrackingObjs;
	vector<TrackingObject*> getTrackingObject();

private:
	vector<TrackingObject*> trackingObjs;
	bool Predicate(cv::Rect r1, cv::Rect r2, double eps);
	int _id = 0;
};

