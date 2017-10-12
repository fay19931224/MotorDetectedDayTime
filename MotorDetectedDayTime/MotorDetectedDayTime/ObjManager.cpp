#include "ObjManager.h"

/*!
*初始化Tracker的各項資訊
* @param frame 為Mat型態，為輸入的原始影像
* @param target 為Rect型態，原始影像灰階後的影像
* @param id 為int型態，為偵測的物件編號
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
* @param color 為Scalar型態，為框框顏色
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
* 取得當前追蹤的信任指數值
* @return 回傳一個float的數值，表示當前追蹤的信任指數值
*/
float TrackingObject::confidence()
{
	return tracker->peak_value;
}

/*!
* 設定Tracker目前的追蹤範圍
* @param newROI為Rect型態，為目前新的ROI位置
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
* 進行目標物件的追蹤，更新追蹤框的資訊，
* 以及砍掉太久沒偵測到或是重疊的框框，並將這些資訊紀錄
* @param frame 為Mat型態，為當前的輸入影像
* @param obj 為vector<cv::Rect>型態，為ROI影像中的物件偵測結果
*/
bool ObjManager::update(cv::Mat &frame, std::vector<cv::Rect> &obj)
{
	/*畫一下*/
	/*for (int i = 0; i < obj.size(); i++)
	{
	rectangle(frame, obj[i], Scalar(0, 0, 0), 2);
	}*/
	//對先前物件做tracking
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


	//刪除太久沒被偵測到的物件
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

	for (int i = 0; i < trackingObjs.size(); i++)//重疊的砍掉
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
* 將追蹤框的結果跟距離畫回影像上
* @param frame 為Mat型態，為輸入影像
* @param color 為Mat型態，為方框的顏色
* @param fusionManager 為Mat型態，此處用來為追蹤框+上應該顯示的各項資訊使用
* @param lidarDistanceData 為vector<long>型態，用來存放當前Frame的Lidar距離資料
* @param roiList 為vector<cv::Rect>型態，為切割過後的ROI影像
* @param classifierType 為ClassiferType型態，表示使用的分類器類型
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
* 根據偵測框與追蹤框的交集面積是否大於其中一方面積的一定比例，判斷目標物是否有出現在追蹤框上
* @param r1 為Rect型態，表示追蹤使用的框
* @param r2 為Rect型態，表示物件偵測使用的框
* @apram eps 為double型態，為計算使用
* @return 回傳true或false，表示是否目標物有出現在追蹤框上
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