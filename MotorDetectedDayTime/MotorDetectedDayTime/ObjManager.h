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
	TrackingObject(cv::Mat &frame, cv::Rect target, int id);
	~TrackingObject();
	void ObjUpdate(cv::Mat &frame);
	void DrawObj(cv::Mat &frame, cv::Scalar& color);
	void updateROI(cv::Rect newROI);
	float confidence();
	cv::Rect TrackingRect;

	int _id = 0;
	bool isNewDetection = true;
	int detectionCount = 0;
	int missCount = 0;
	bool isTracking = false;
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
	bool update(cv::Mat &frame, std::vector<cv::Rect> &obj);
	void draw(cv::Mat& frame, cv::Scalar& color);
	//void draw(cv::Mat& frame, cv::Scalar& color, FusionManager& fusionManager, vector<long>& lidarDistanceData, vector<Rect> &roiList, ClassiferType classifierType);
	~ObjManager();
	vector<Rect> _restTrackingObjs;

private:
	vector<TrackingObject*> trackingObjs;
	bool Predicate(cv::Rect r1, cv::Rect r2, double eps);
	bool IsRectCross(cv::Rect rect1, cv::Rect rect2);
	int _id = 0;
};

