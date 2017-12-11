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
* ��class�Ψ�Ū���v����Lidar��ơA�ïണ�ѳ�iframe�������浧Lidar���
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