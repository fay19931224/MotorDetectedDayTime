#ifndef VIDEOREADER_H
#define VIDEOREADER_H

#include <string>
#include <vector>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <mutex>

using namespace std;
using cv::VideoCapture;
using cv::Mat;
using cv::Size;

class VideoReader
{
protected:
	int _dataQuantity;
	int _cameraFPS;
	Mat _currentFrame;	
	Mat _currentGrayFrame;
	VideoCapture _cameraVideo;
public:
	VideoReader(string videoFileName);
	virtual ~VideoReader();
	int GetDataQuantity();
	int GetCameraFPS();
	void setFrame();
	Mat getFrame();
	Mat getGrayFrame();
	virtual string StartRead();
	virtual void RequestData(Mat &frame);
	Size getVideoSize();	
};

#endif