#include "ObjManager.h"

/*!
*��l��Tracker���U����T
* @param frame ��Mat���A�A����J����l�v��
* @param target ��Rect���A�A��l�v���Ƕ��᪺�v��
* @param id ��int���A�A������������s��
*/
TrackingObject::TrackingObject(cv::Mat &frame, cv::Rect target, int id, Scalar color)
{
	TrackingRect = target;
	_id = id;
	_color = color;	
	tracker = new KCFTracker(true, false, true, true);
	tracker->init(target, frame);	
	initRect = target;

}

TrackingObject::~TrackingObject()
{	
	delete tracker;
}

/*!
* �ϥο�J�v����s�l�ܮ�
* @param frame ��Mat���A�A����J�v��
*/
void TrackingObject::ObjUpdate(cv::Mat &frame)
{
	TrackingRect = tracker->update(frame);
}

/*!
* �N�l�ܮ���ܩ��J�v���W
* @param frame ��Mat���A�A����J�v��
* @param color ��Scalar���A�A���خ��C��
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
* ���o��e�l�ܪ��H�����ƭ�
* @return �^�Ǥ@��float���ƭȡA��ܷ�e�l�ܪ��H�����ƭ�
*/
float TrackingObject::confidence()
{
	return tracker->peak_value;
}

/*!
* �]�wTracker�ثe���l�ܽd��
* @param newROI��Rect���A�A���ثe�s��ROI��m
*/
void TrackingObject::updateROI(cv::Rect newROI)
{
	tracker->setROI(newROI);
}