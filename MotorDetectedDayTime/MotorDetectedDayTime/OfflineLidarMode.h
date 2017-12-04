#pragma once


#include "DataReader.h"
#include "SvmClassifier.h"
#include <Windows.h>
#include <time.h>
#include "Mode.h"
class OfflineLidarMode:public Mode
{
public:
	OfflineLidarMode();
	virtual ~OfflineLidarMode();	
	void Run();
	Rect adjustROI(Mat, Rect);
private:
	string _videoFileName;
	int _filterRoiDistance;
	int _waitKeySec;
	int _waitKeyChoosen;
	FusionType _type;
	bool WaitKey();
	void Detect(Mat &frame, Mat &grayFrame, int count);
};

OfflineLidarMode::OfflineLidarMode()
{
}

OfflineLidarMode::~OfflineLidarMode()
{
}