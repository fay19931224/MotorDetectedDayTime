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
class DataReader :public VideoReader
{
private:
	ifstream _lidarTextFile;
	int GetLidarDataCount();
	//void RetrieveLidarDataFromText(string &text, string symbol, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData);

public:
	DataReader(string videoFileName, string lidarFileName);
	~DataReader();
	string StartRead();
	//void RequestOneData(Mat &frame, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData, string &lidarHeader);
};

#endif