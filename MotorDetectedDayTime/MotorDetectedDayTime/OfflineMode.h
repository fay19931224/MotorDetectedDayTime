#ifndef OFFLINE_MODE_H
#define OFFLINE_MODE_H

#include "DataReader.h"
#include "SvmClassifier.h"
#include "HeadSVMDetecter.h"
#include <Windows.h>
#include <time.h>
#include "Mode.h"

/*!
* 此class為取得影像以及Lidar資料後，根據設定的距離在影像上進行ROI的切割以及物件的偵測及追蹤
*/
using cv::Rect;
using cv::Mat;
using cv::waitKey;
using cv::VideoWriter;
using cv::Vec3f;
using cv::destroyAllWindows;
class OfflineMode : public Mode
{
private:
	string _videoFileName;
	int _filterRoiDistance;
	int _waitKeySec;
	int _waitKeyChoosen;
	FusionType _type;
	bool WaitKey();
	void Detect(Mat &frame, Mat &grayFrame,int count);
	void OnGrab(void *info);
public:
	OfflineMode(string videoFileName, FusionType type, int currentModelType = 0);
	virtual ~OfflineMode();	
	void Run();
	Rect adjustROI(Mat, Rect);
};

#endif