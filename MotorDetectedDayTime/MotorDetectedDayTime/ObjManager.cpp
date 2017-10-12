#include "ObjManager.h"

/*!
*��l��Tracker���U����T
* @param frame ��Mat���A�A����J����l�v��
* @param target ��Rect���A�A��l�v���Ƕ��᪺�v��
* @param id ��int���A�A������������s��
*/
TrackingObject::TrackingObject(cv::Mat &frame, cv::Rect target, int id)
{
	TrackingRect = target;
	_id = id;
	tracker = new KCFTracker(0, 1, true, 0);
	tracker->init(target, frame);
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
void TrackingObject::DrawObj(cv::Mat &frame, cv::Scalar& color)
{
	/*if (0 <= TrackingRect.x && TrackingRect.x + TrackingRect.width < frame.cols && 0 <= TrackingRect.y && TrackingRect.y + TrackingRect.height < frame.rows)
	{
	imwrite(to_string(AAA++) + ".jpg", Mat(frame, TrackingRect));
	}*/
	cv::rectangle(frame, TrackingRect, color, 2);
	/*stringstream ss;
	ss << name;
	imwrite(ss.str(), frame(TrackingRect));*/

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

ObjManager::ObjManager()
{
}


ObjManager::~ObjManager()
{
}

/*!
* �i��ؼЪ��󪺰l�ܡA��s�l�ܮت���T�A
* �H�ά屼�Ӥ[�S������άO���|���خءA�ñN�o�Ǹ�T����
* @param frame ��Mat���A�A����e����J�v��
* @param obj ��vector<cv::Rect>���A�A��ROI�v���������󰻴����G
*/
bool ObjManager::update(cv::Mat &frame, std::vector<cv::Rect> &obj)
{
	/*�e�@�U*/
	/*for (int i = 0; i < obj.size(); i++)
	{
	rectangle(frame, obj[i], Scalar(0, 0, 0), 2);
	}*/
	//����e����tracking
	for (int i = 0; i < trackingObjs.size(); i++)
	{
		trackingObjs[i]->isNewDetection = false;
		trackingObjs[i]->ObjUpdate(frame);
	}
	for (int i = 0; i < obj.size(); i++)
	{
		bool match = false;
		for (int j = 0; j < trackingObjs.size(); j++)
		{
			if (Predicate(trackingObjs[j]->TrackingRect, obj[i], 0.5))
			{
				trackingObjs[j]->detectionCount++;
				trackingObjs[j]->missCount = 0;
				trackingObjs[j]->isNewDetection = true;
				match = true;
				if (trackingObjs[j]->detectionCount >= 3)
				{
					trackingObjs[j]->isTracking = true;
				}

				//adjust tracking rect
				trackingObjs[j]->TrackingRect.x = (trackingObjs[j]->TrackingRect.x + obj[i].x) / 2;
				trackingObjs[j]->TrackingRect.y = (trackingObjs[j]->TrackingRect.y + obj[i].y) / 2;
				trackingObjs[j]->TrackingRect.width = (trackingObjs[j]->TrackingRect.width + obj[i].width) / 2;
				trackingObjs[j]->TrackingRect.height = (trackingObjs[j]->TrackingRect.height + obj[i].height) / 2;

				trackingObjs[j]->updateROI(trackingObjs[j]->TrackingRect);

			}
		}
		if (!match)
		{
			TrackingObject* t = new TrackingObject(frame, obj[i], _id++);
			t->isTracking = false;
			trackingObjs.push_back(t);
		}
	}

	for (int i = 0; i < trackingObjs.size(); i++)
	{
		//float confidence = trackingObjs[i]->confidence();
		if (trackingObjs[i]->isNewDetection)
		{
			trackingObjs[i]->detectionCount++;
			trackingObjs[i]->missCount = 0;
			if (trackingObjs[i]->detectionCount >= 2)
			{
				trackingObjs[i]->isTracking = true;
			}
		}
	}


	//�R���Ӥ[�S�Q�����쪺����
	for (int i = 0; i < trackingObjs.size(); i++)
	{
		if (!trackingObjs[i]->isNewDetection)
		{
			trackingObjs[i]->missCount++;
		}

		if (trackingObjs[i]->missCount >= 3)
		{
			float confidence = trackingObjs[i]->confidence();
			if (trackingObjs[i]->missCount < 30 && confidence < 0.7)
			{
				TrackingObject* temp = trackingObjs[i];

				_restTrackingObjs.push_back(trackingObjs[i]->TrackingRect);
				std::cout << "delete missCount! " << temp->_id << std::endl;
				trackingObjs.erase(find(trackingObjs.begin(), trackingObjs.end(), temp));
				delete temp;
			}
			else if (trackingObjs[i]->missCount >= 30)
			{
				float confidence = trackingObjs[i]->confidence();
				//if (confidence < 0.7)
				//{
				TrackingObject* temp = trackingObjs[i];
				_restTrackingObjs.push_back(trackingObjs[i]->TrackingRect);
				std::cout << "delete missCount " << temp->_id << std::endl;
				trackingObjs.erase(find(trackingObjs.begin(), trackingObjs.end(), temp));
				delete temp;
				//}
			}
		}
	}

	for (int i = 0; i < trackingObjs.size(); i++)//���|���屼
	{
		if (trackingObjs[i]->isTracking)
		{
			for (int j = 0; j < trackingObjs.size(); j++)
			{
				if (i != j && trackingObjs[j]->isTracking && Predicate(trackingObjs[j]->TrackingRect, trackingObjs[i]->TrackingRect, 0.5))//wanf
				{
					TrackingObject* temp = trackingObjs[i];
					std::cout << "delete cross " << temp->_id << std::endl;
					trackingObjs.erase(find(trackingObjs.begin(), trackingObjs.end(), temp));
					delete temp;
					break;
				}
			}
		}
	}
	return trackingObjs.size();
}

void ObjManager::draw(cv::Mat& frame, cv::Scalar& color)
{
	for (int i = 0; i < trackingObjs.size(); i++)
	{
		if (trackingObjs[i]->isTracking)
		{
			trackingObjs[i]->DrawObj(frame, color);
		}
	}
}

/*!
* �N�l�ܮت����G��Z���e�^�v���W
* @param frame ��Mat���A�A����J�v��
* @param color ��Mat���A�A����ت��C��
* @param fusionManager ��Mat���A�A���B�ΨӬ��l�ܮ�+�W������ܪ��U����T�ϥ�
* @param lidarDistanceData ��vector<long>���A�A�ΨӦs���eFrame��Lidar�Z�����
* @param roiList ��vector<cv::Rect>���A�A�����ιL�᪺ROI�v��
* @param classifierType ��ClassiferType���A�A��ܨϥΪ�����������
*/
/*void ObjManager::draw(cv::Mat& frame, cv::Scalar& color, FusionManager& fusionManager, vector<long>& lidarDistanceData, vector<Rect> &roiList, ClassiferType classifierType)
{
	for (int i = 0; i < trackingObjs.size(); i++)
	{
		trackingObjs[i]->DrawObj(frame, color);
		for (int j = 0; j < roiList.size(); j++)
		{
			float distance = fusionManager.RequestDistance(frame, trackingObjs[i]->TrackingRect, lidarDistanceData);
			fusionManager.AddInformationOnObject(frame, trackingObjs[i]->TrackingRect, classifierType, distance, color);
		}
	}
}*/

/*!
* �ھڰ����ػP�l�ܮت��涰���n�O�_�j��䤤�@�譱�n���@�w��ҡA�P�_�ؼЪ��O�_���X�{�b�l�ܮؤW
* @param r1 ��Rect���A�A��ܰl�ܨϥΪ���
* @param r2 ��Rect���A�A��ܪ��󰻴��ϥΪ���
* @apram eps ��double���A�A���p��ϥ�
* @return �^��true��false�A��ܬO�_�ؼЪ����X�{�b�l�ܮؤW
*/
bool ObjManager::Predicate(cv::Rect r1, cv::Rect r2, double eps)
{
	cv::Rect temp = r1 & r2;
	if ((temp.area() > r1.area() * eps) || temp.area() > r2.area() * eps)
	{
		return true;
	}
	else
	{
		return false;
	}
}