#ifndef FUSION_MANAGER_H
#define FUSION_MANAGER_H

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <vector>
#include "ClassifierType.h"

using cv::Rect;
using cv::Mat;
using cv::FONT_HERSHEY_COMPLEX;
using namespace std;

class FusionManager
{
private:
	enum DRIVING_STATE
	{
		EMPTY,
		LOW,
		MID,
		HIGH
	};

	const float FONT_SCALE = 0.5;
	const int FONT_FACE = FONT_HERSHEY_COMPLEX;
	const int TEXT_LINE_SPACE = 2;
	const int FONT_THICKNESS = 1;

	float  CAMERA_ANGLE_ST;
	float  CAMERA_ANGLE_ED;
	float  CAMERA_ANGLE_TOTAL;
	float  LIDAR_ANGLE_ST;
	float  LIDAR_ANGLE_ED;
	float  LIDAR_ANGLE_TOTAL;

	float  LOW_THRES_DISTNACE;
	float  MID_THRES_DISTANCE;
	DRIVING_STATE _currentState = DRIVING_STATE::EMPTY;

	float dataLen;
	float total;
	float st;
	float ed ;
	

	float CalculateDistance(Mat &frame, int st, int ed, vector<long> &lidarDistanceData);
	vector<long> _lidarDistanceData;
public:
	FusionManager();
	 ~FusionManager();
	float RequestDistance(Mat &frame, Rect& roi);
	pair<int, int> getReadLidarPosition();
	void AddInformationOnObject(Mat &frame, Rect& roi, ClassiferType classifierType, float distance, Scalar textColor);
	vector<Rect> FilterPosibleArea(Mat &frame, int stDistance, int edDistance, vector<long> &lidarDistanceData, int minimumRoiWidth);
	void ShowLidarBar(Mat &frame, vector<long> &lidarDistanceData);
	void InititalizeDistanceLimit(int lowDistance, int midDistance);
	void SyncLidarAndCamera(int lidarAngleSt = -95, int lidarAngleEd = 95, int cameraAngleSt = -30, int cameraAngleEd = 30);
	void ClearState();
	void ShowState(Mat &frame);
	vector<long> getLidarDistanceData();
	void setLidarDistanceData(vector<long> lidarDistanceData);
};

#endif