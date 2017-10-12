#ifndef VIDEOREADER_H
#define VIDEOREADER_H

#include <string>
#include <vector>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace std;
using cv::VideoCapture;
using cv::Mat;
using cv::Size;

class VideoReader
{
protected:
	int _dataQuantity;
	int _cameraFPS;
	VideoCapture _cameraVideo;
public:
	VideoReader(string videoFileName);
	virtual ~VideoReader();
	int GetDataQuantity();
	int GetCameraFPS();
	string StartRead();
	void RequestOneData(Mat &frame);
	Size getVideoSize();
};

#endif