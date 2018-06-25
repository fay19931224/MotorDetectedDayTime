#ifndef OFFLINE_MODE_H
#define OFFLINE_MODE_H

#include "LidarReader.h"
#include "SvmClassifier.h"
#include "HeadDetecter.h"
#include <time.h>
#include "Mode.h"
/*!
* ��class�����o�v���H��Lidar��ƫ�A�ھڳ]�w���Z���b�v���W�i��ROI�����ΥH�Ϊ��󪺰����ΰl��
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
	string _lidarFileName;
	int _filterRoiDistance;
	int _waitKeySec;
	int _waitKeyChoosen;
	FusionType _type;
	bool WaitKey();
	void Detect(Mat &frame, Mat &grayFrame,int count);
	vector<long> lidarDistanceData;
	vector<unsigned short>  lidarSignalData;
	
	bool motobackfrontFlag = false;
	bool motosideCountFlag = false;
	bool pedfrontCountFlag = false;

public:	
	
	Mat frame;
	OfflineMode(string videoFileName, string lidarFileName, FusionType type, int currentModelType);
	virtual ~OfflineMode();	
	void Run();
	void RunThread();	
};

#endif