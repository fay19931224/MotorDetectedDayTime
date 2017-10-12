#include "DataReader.h"

DataReader::DataReader(string videoFileName, string lidarFileName) :VideoReader(videoFileName)
{
	_lidarTextFile.open(lidarFileName, ios::in);
}

DataReader::~DataReader()
{
	_lidarTextFile.close();
}

/*!
* 計算Lidar資料總共有幾筆
*/
int DataReader::GetLidarDataCount()
{
	int count = 0;
	while (!_lidarTextFile.eof())
	{
		string headerText;
		getline(_lidarTextFile, headerText);
		string quantityText = headerText.substr(0, headerText.find(" "));
		int quantity = atoi(quantityText.c_str());
		for (int i = 0; i < quantity; i++)
		{
			string data;
			getline(_lidarTextFile, data);
		}
		if (quantity > 0)
		{
			count++;
		}
	}
	_lidarTextFile.clear();
	_lidarTextFile.seekg(0, ios::beg);
	return count;
}

/*!
* 從txt讀取Lidar資料，每筆資料是用1個距離資料以及一個光反射強度值組成
*/
/*void DataReader::RetrieveLidarDataFromText(string &text, string symbol, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData)
{
	int per = 0;
	int beforeValue;
	char *temp = strdup(text.c_str());
	char *garbege;
	strtok_s(temp, symbol.c_str(), &garbege);
	while (temp != NULL && per < 2)
	{
		if (per == 0)
		{
			beforeValue = atoi(temp);
		}
		else if (per == 1)
		{
			lidarDistanceData.push_back(beforeValue);
			lidarSignalData.push_back(atoi(temp));
		}
		temp = strtok_s(NULL, symbol.c_str(), &garbege);
		per++;
	}
	free(temp);
}*/

/*!
* 讀取影片及Lidar資料，如影像或是LIDAR資料讀取發生問題則返回錯誤訊息
* @param videlFileName 為string 類型，為要偵測的影片名稱
* @param lidarFileName 為string 類型，為要偵測的影片對應的Lidar資料名稱
* @return 回傳錯誤訊息，如無任何錯誤則回傳包含frame數以及Lidar資料數的成功訊息
*/
string DataReader::StartRead()
{
	VideoReader::StartRead();
	if (!_lidarTextFile)
	{
		return "LidarTextFile Error";
	}
	return "File open success, total " + to_string(_dataQuantity) + " frame, " + to_string(GetLidarDataCount()) + " lidar data";
}

/*!
* 取得一張frame以及對應的一串Lidar資料
* @param frame 為Mat型態，為原始輸入影像
* @param lidarDistanceData 為vector<ling>型態，用來存放每筆Lidar資料中共1521個角度的距離
* @param lidarSignalData 為vector<ling>型態，用來存放每筆Lidar資料中共1521個角度的訊號強度
* @param lidarHeader 為string型態，用來切割Lidar資料以便取得每筆資料數量
*/
/*void DataReader::RequestOneData(Mat &frame, vector<long>& lidarDistanceData, vector<unsigned short>& lidarSignalData, string &lidarHeader)
{
	getline(_lidarTextFile, lidarHeader);
	string quantityText = lidarHeader.substr(0, lidarHeader.find(" "));
	int quantity = atoi(quantityText.c_str());
	for (int i = 0; i < quantity; i++)
	{
		string dataText;
		getline(_lidarTextFile, dataText);
		RetrieveLidarDataFromText(dataText, " ,", lidarDistanceData, lidarSignalData);
	}
	_cameraVideo.read(frame);
}*/