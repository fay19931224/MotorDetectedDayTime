#include "ObjManager.h"

/*!
*初始化Tracker的各項資訊
* @param frame 為Mat型態，為輸入的原始影像
* @param target 為Rect型態，原始影像灰階後的影像
* @param id 為int型態，為偵測的物件編號
*/
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

/*!
* 使用輸入影像更新追蹤框
* @param frame 為Mat型態，為輸入影像
*/
void TrackingObject::ObjUpdate(cv::Mat &frame)
{
	TrackingRect = tracker->update(frame);
}

/*!
* 將追蹤框顯示於輸入影像上
* @param frame 為Mat型態，為輸入影像
*/
void TrackingObject::DrawObj(cv::Mat &frame)
{	
	cv::rectangle(frame, TrackingRect, _color, 2);	
}

Rect TrackingObject::getROI()
{
	return TrackingRect;
}

/*!
* 取得當前追蹤的信任指數值
* @return 回傳一個float的數值，表示當前追蹤的信任指數值
*/
float TrackingObject::confidence()
{
	return tracker->peak_value;
}