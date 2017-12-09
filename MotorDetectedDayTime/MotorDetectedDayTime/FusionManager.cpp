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
	int dataLen = _lidarDistanceData.size();//共幾筆資料
	int total = CAMERA_ANGLE_TOTAL*dataLen / LIDAR_ANGLE_TOTAL;
	int st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*dataLen / LIDAR_ANGLE_TOTAL;
	int ed = st + total;
	int posEd = roi.x + roi.width;	
	vector<pair<int, long>> leadList;
	for (int i = roi.x; i < posEd; i++)
	{
		int index = i*total / frame.cols;
		leadList.push_back(make_pair(index, _lidarDistanceData[index]));
	}
	
	/*sort(leadList.begin(), leadList.end(), isComp);
	float sumDistance = 0.0f;
	int per = leadList.size() / 10;
	for (int i = 0; i < per; i++)
	{
	sumDistance += leadList[i].second;
	}

	DRIVING_STATE state = DRIVING_STATE::EMPTY;
	float distance = sumDistance / per;*/
	//sort(leadList.begin(), leadList.end(), isComp);
	sort(leadList.begin(), leadList.end(), isComp);
	float sumDistance = 0.0f;
	int size = leadList.size();
	for (int i = size * 0.2; i < size * 0.8; i++)
	{
		sumDistance += leadList[i].second;
	}

	DRIVING_STATE state = DRIVING_STATE::EMPTY;
	float distance = sumDistance / (size * 0.6);

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
}

vector<long> FusionManager::getLidarDistanceData()
{
	return _lidarDistanceData;
}

void FusionManager::setLidarDistanceData(vector<long> lidarDistanceData)
{
	_lidarDistanceData = lidarDistanceData;
}
