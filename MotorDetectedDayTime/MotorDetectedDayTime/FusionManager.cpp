#include "FusionManager.h"

FusionManager::FusionManager()
{
}

FusionManager::~FusionManager()
{
}

bool isComp(pair<int, long> &p1, pair<int, long>&p2)
{
	return p1.second < p2.second;
}


float FusionManager::RequestDistance(Mat & frame, Rect & roi)
{		
	
	

	float posEd = roi.x + roi.width;
	//vector<pair<int, long>> leadList;
	/*Mat temp(120, dataLen, CV_8UC3, Scalar(255, 255, 255));
	for (int i = 0; i<dataLen; i++)
	{
		cv::line(temp, CvPoint(i, temp.rows - 1), CvPoint(i, (temp.rows - 1) - _lidarDistanceData[dataLen - i - 1] / 1000.0), Scalar(255, 0, 0), 1, 8, 0);
	}*/
	int Min = INT_MAX;
	for (int i = roi.x; i < posEd; i++)
	{
		int index = i * total / frame.cols+st;
		if (Min > _lidarDistanceData[dataLen - index - 1]) 
		{
			Min = _lidarDistanceData[dataLen - index - 1];
		}
		//leadList.push_back(make_pair(index, _lidarDistanceData[dataLen - index - 1]));		
	}		
	//sort(leadList.begin(), leadList.end(), isComp);		
	float distance = Min;

	DRIVING_STATE state = DRIVING_STATE::EMPTY;	
	if (distance < LOW_THRES_DISTNACE)
	{
		state = DRIVING_STATE::HIGH;
	}
	else if (distance < MID_THRES_DISTANCE)
	{
		state = DRIVING_STATE::MID;
	}
	else
	{
		state = DRIVING_STATE::LOW;
	}

	if (state > _currentState)
	{
		_currentState = state;
	}
	return distance;
}

//pair<int, int> FusionManager::getReadLidarPosition()
//{
//	return pair<int, int>(st,ed);
//}

void FusionManager::InititalizeDistanceLimit(int lowDistance, int midDistance)
{
	LOW_THRES_DISTNACE = lowDistance;
	MID_THRES_DISTANCE = midDistance;
}

void FusionManager::SyncLidarAndCamera(int lidarAngleSt, int lidarAngleEd, int cameraAngleSt, int cameraAngleEd)
{
	LIDAR_ANGLE_ST = lidarAngleSt;
	LIDAR_ANGLE_ED = lidarAngleEd;
	LIDAR_ANGLE_TOTAL = LIDAR_ANGLE_ED - LIDAR_ANGLE_ST;
	CAMERA_ANGLE_ST = cameraAngleSt;
	CAMERA_ANGLE_ED = cameraAngleEd;
	CAMERA_ANGLE_TOTAL = CAMERA_ANGLE_ED - CAMERA_ANGLE_ST;

	dataLen = 1521;//共幾筆資料
	total = CAMERA_ANGLE_TOTAL*(dataLen / LIDAR_ANGLE_TOTAL);//換算為相機後共幾筆資料
	st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*(dataLen / LIDAR_ANGLE_TOTAL);
	ed = st + total;
}

vector<long> FusionManager::getLidarDistanceData()
{
	return _lidarDistanceData;
}

void FusionManager::setLidarDistanceData(vector<long> lidarDistanceData)
{
	_lidarDistanceData = lidarDistanceData;
}
