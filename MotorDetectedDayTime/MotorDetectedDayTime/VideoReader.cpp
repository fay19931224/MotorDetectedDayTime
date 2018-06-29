#include "VideoReader.h"

VideoReader::VideoReader(string videoFileName)
{
	_cameraVideo.open(videoFileName);
}

VideoReader::~VideoReader()
{

}

int VideoReader::GetDataQuantity()
{
	return _dataQuantity;
}

int VideoReader::GetCameraFPS()
{
	return _cameraFPS;
}

void VideoReader::setFrame()
{
	_cameraVideo.read(_currentFrame);
	cvtColor(_currentFrame, _currentGrayFrame, CV_BGR2GRAY);
}

string VideoReader::StartRead()
{
	std::cout << "start read video" << std::endl;
	if (!_cameraVideo.isOpened())
	{
		return "Camera Error";
	}
	_dataQuantity = _cameraVideo.get(CV_CAP_PROP_FRAME_COUNT);
	_cameraFPS = _cameraVideo.get(CV_CAP_PROP_FPS);
	return "File open success, total " + to_string(_dataQuantity) + " frame";
}

void VideoReader::RequestData(Mat &frame)
{
	_cameraVideo.read(frame);
}

Size VideoReader::getVideoSize()
{
	return Size(_cameraVideo.get(CV_CAP_PROP_FRAME_WIDTH), _cameraVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
}