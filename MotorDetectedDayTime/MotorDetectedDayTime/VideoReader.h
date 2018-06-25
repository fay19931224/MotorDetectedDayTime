#ifndef VIDEOREADER_H
#define VIDEOREADER_H

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2\highgui\highgui.hpp>
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
	Mat _currentFrame;	
	Mat _currentGrayFrame;
	VideoCapture _cameraVideo;
public:
	VideoReader(string videoFileName);
	virtual ~VideoReader();
	int GetDataQuantity();
	int GetCameraFPS();
	void setFrame();
	
	virtual string StartRead();
	virtual void RequestData(Mat &frame);
	Size getVideoSize();	
};

#endif