#ifndef DATA_READER_H
#define DATA_READER_H

#include "VideoReader.h"
#include <string>
#include <vector>
#include <fstream>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace std;



/*!
* 此class用來讀取影像及Lidar資料，並能提供單張frame對應的單筆Lidar資料
*/
class LidarReader :public VideoReader
{
private:
	ifstream _lidarTextFile;
	int GetLidarDataCount();
	int _lidarDataLength=1521;
	//void RetrieveLidarDataFromText(string &text, string symbol, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData);

public:
	LidarReader(string videoFileName, string lidarFileName);
	~LidarReader();
	string StartRead();
	void RequestData(Mat &frame, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData, int start, int end);
	void RetrieveLidarDataFromText(string &text, string symbol, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData);
	int getLidarDataLength();
};

#endif