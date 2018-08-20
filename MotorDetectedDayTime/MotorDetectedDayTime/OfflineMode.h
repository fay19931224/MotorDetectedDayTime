#ifndef OFFLINE_MODE_H
#define OFFLINE_MODE_H

#include "VideoReader.h"
#include "SvmClassifier.h"
#include "HeadDetecter.h"
#include <time.h>

using cv::Rect;
using cv::Mat;
using cv::waitKey;
using cv::VideoWriter;
using cv::Vec3f;
using cv::destroyAllWindows;
class OfflineMode
{
private:
	string _videoFileName;
	int _waitKeySec;
	int _waitKeyChoosen;
	FusionType _type;
	bool WaitKey();
	void Detect(Mat &frame, Mat &grayFrame,int count);

	vector<Classifier*> _classifierList;
	bool motobackfrontFlag = false;
	bool motosideCountFlag = false;
	bool pedfrontCountFlag = false;
	int mode = 0;
	vector<Rect> frontbackobject;
	vector<Rect> sideobject;
	vector<Rect> pedobject;
public:		
	Mat frame;
	OfflineMode(string videoFileName, FusionType type, int currentModelType);
	~OfflineMode();	
	void Run();	
	void RunCompare();
};

#endif