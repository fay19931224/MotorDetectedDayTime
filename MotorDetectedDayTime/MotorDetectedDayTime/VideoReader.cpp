#include "VideoReader.h"

VideoReader::VideoReader(string videoFileName)
{
	_cameraVideo.open(videoFileName);
}

VideoReader::~VideoReader()
{

}

/*!
*取得總Frame數
*/
int VideoReader::GetDataQuantity()
{
	return _dataQuantity;
}

int VideoReader::GetCameraFPS()
{
	return _cameraFPS;
}

/*!
* 讀取影片資料，如影像資料讀取發生問題則返回錯誤訊息
* @param videlFileName 為string 類型，為要偵測的影片名稱
* @return 回傳錯誤訊息，如無任何錯誤則回傳包含frame數以及Lidar資料數的成功訊息
*/
string VideoReader::StartRead()
{
	if (!_cameraVideo.isOpened())
	{
		return "Camera Error";
	}
	_dataQuantity = _cameraVideo.get(CV_CAP_PROP_FRAME_COUNT);
	_cameraFPS = _cameraVideo.get(CV_CAP_PROP_FPS);
	return "File open success, total " + to_string(_dataQuantity) + " frame, ";
}

/*!
* 取得一張frame以及對應的一串Lidar資料
* @param frame 為Mat型態，為原始輸入影像
*/
void VideoReader::RequestOneData(Mat &frame)
{
	_cameraVideo.read(frame);
}

Size VideoReader::getVideoSize()
{
	return Size(_cameraVideo.get(CV_CAP_PROP_FRAME_WIDTH), _cameraVideo.get(CV_CAP_PROP_FRAME_HEIGHT));
}