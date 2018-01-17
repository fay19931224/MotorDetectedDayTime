#include "FusionManager.h"

FusionManager::FusionManager()
{
}

FusionManager::~FusionManager()
{
}

bool isComp(pair<int, long> &p1, pair<int, long>&p2)
{
	return p1.second < p2.second;
}


float FusionManager::RequestDistance(Mat & frame, Rect & roi)
{			
	float posEd = roi.x + roi.width;	
	/*Mat temp(120, dataLen, CV_8UC3, Scalar(255, 255, 255));	
	for (int i = 0; i<dataLen; i++)
	{
		cv::line(temp, CvPoint(i, temp.rows - 1), CvPoint(i, (temp.rows - 1) - _lidarDistanceData[dataLen - i - 1] / 1000.0), Scalar(255, 0, 0), 1, 8, 0);
	}*/
	
	int Min = INT_MAX;
	for (int i = roi.x; i < posEd; i++)
	{		
		int index = i * total / frame.cols;		
		if (Min > _lidarDistanceData[index])
		{
			Min = _lidarDistanceData[index];
		}	
	}
	
	//imshow("d", temp);
	float distance = Min;
	/*DRIVING_STATE state = DRIVING_STATE::EMPTY;	
	if (distance < LOW_THRES_DISTNACE)
	{
		state = DRIVING_STATE::HIGH;
	}
	else if (distance < MID_THRES_DISTANCE)
	{
		state = DRIVING_STATE::MID;
	}
	else
	{
		state = DRIVING_STATE::LOW;
	}

	if (state > _currentState)
	{
		_currentState = state;
	}*/
	return distance;
}

Mat FusionManager::showLidarImformation(Mat frame)
{	
	Mat temp(120, frame.cols, CV_8UC3, Scalar(255, 255, 255));

	for (int i = 0; i<frame.cols; i++)
	{		
		int index = i * total / frame.cols;	
		long t = _lidarDistanceData[index] / 1000.0;		
		if (t < 40)
		{
			cv::line(temp, CvPoint(i, temp.rows - 1), CvPoint(i, (temp.rows - 1) - t), Scalar(255, 0, 0), 1, 8, 0);
		}else if (t < 80) 
		{
			cv::line(temp, CvPoint(i, temp.rows - 1), CvPoint(i, (temp.rows - 1) - t), Scalar(0, 255, 0), 1, 8, 0);
		}
	}		
	//cin.get();
	//imshow("distance", temp);	
	return temp;
}

Mat FusionManager::showWholeLidarImformation(Mat frame)
{
	//Mat temp(120, _lidarDistanceData.size(), CV_8UC3, Scalar(255, 255, 255));	
	Mat lidarDis(120, frame.cols, CV_8UC3, Scalar(255, 255, 255));
	Mat lidarDisL(120, frame.cols /2, CV_8UC3, Scalar(255, 255, 255));
	Mat lidarDisR(120, frame.cols / 2, CV_8UC3, Scalar(255, 255, 255));
	
	vector<int> left(32, 0);
	vector<int> right(32, 0);

	double angle = 0;
	double PI = 3.1415926;
	for (int i =0; i< _lidarDistanceData.size()/2; i++)
	{
		double r = _lidarDistanceData[i] / 1000.0;
		//cout << angle << " " << ":" << r << endl;
		int x = r*sin(angle*PI / 180.0)*3.0;
		int y = r*cos(angle*PI / 180.0)*3.0;
		Scalar color;
		if (r < 40)
		{						
			color = Scalar(255, 0, 0);
			cv::line(lidarDisL, CvPoint(x, 0), CvPoint(x, y), color, 1, 8, 0);
		}
		else if (r < 120)
		{			
			color = Scalar(0, 255, 0);
			cv::line(lidarDisL, CvPoint(x, 0), CvPoint(x, y), color, 1, 8, 0);
		}
		else {
			color = Scalar(0, 0, 255);
			cv::line(lidarDisL, CvPoint(x, 0), CvPoint(x, y), color, 1, 8, 0);
		}
		
		angle += 0.125;
	}		
	angle = 0;

	Mat lidarDisR2(120, frame.cols / 2, CV_8UC3, Scalar(255, 255, 255));
	//最寬76
	//double xRange = frame.cols / 2;
	for (int i = _lidarDistanceData.size()/2; i< _lidarDistanceData.size(); i++)	
	{				
		double r = _lidarDistanceData[i] / 1000.0;
		//cout << angle<<" "<< i<<":"<< r << endl;
	
		int x = r*sin(angle*PI / 180.0);
		int y = r*cos(angle*PI / 180.0);
		
		
		Scalar color;
		CvPoint p1 = CvPoint(0, 0);
		CvPoint p2 = CvPoint(x, y);

		
		cv::line(lidarDisR2, CvPoint(200, 0), CvPoint(200, 5), color, 1, 8, 0);
		if (y<1)
		{	
			cout << y << endl;
			cin.get();
		}
		if (r < 40)
		{
			color = Scalar(255, 0, 0);
			cv::line(lidarDisR, p1,p2 , color, 1, CV_AA, 0);
			cv::line(lidarDisR2, CvPoint(x, 0), p2, color, 1, CV_AA, 0);
		}
		else if (r < 120)
		{
			/*if (y < 0)
			{
			cout << angle << endl;
			cout << "r:" << r << endl;
			cout << "x:" << x << endl;
			cout << "y:" << y << endl;
			cout << endl;
			cin.get();
			}*/
			color = Scalar(0, 255, 0);
			cv::line(lidarDisR, p1, p2, color, 1, CV_AA, 0);
			cv::line(lidarDisR2, CvPoint(x, 0), p2, color, 1, CV_AA, 0);
		}
		else 
		{
			color = Scalar(0, 0, 255);
			cv::line(lidarDisR, p1, p2, color, 1, CV_AA, 0);
			cv::line(lidarDisR2, CvPoint(x, 0), p2, color, 1, CV_AA, 0);
		}		
		angle +=0.125;
	}
	flip(lidarDisR, lidarDisR, 0);
	flip(lidarDisR2, lidarDisR2, 0);
	//flip(lidarDisL, lidarDisL, 0);
	//flip(lidarDisL, lidarDisL, 1);
	cv::hconcat(lidarDisL, lidarDisR, lidarDis);
	//cv::resize(lidarDis, lidarDis,Size(320,32), CV_INTER_CUBIC);
	//cin.get();
	imshow("datadistance", lidarDisR);
	imshow("datadistance2", lidarDisR2);
	return lidarDis;
}

pair<int, int> FusionManager::getReadLidarPosition()
{
	return pair<int, int>(st,ed);
}

void FusionManager::InititalizeDistanceLimit(int lowDistance, int midDistance)
{
	LOW_THRES_DISTNACE = lowDistance;
	MID_THRES_DISTANCE = midDistance;
}

void FusionManager::SyncLidarAndCamera(int lidarAngleSt, int lidarAngleEd, int cameraAngleSt, int cameraAngleEd)
{
	LIDAR_ANGLE_ST = lidarAngleSt;
	LIDAR_ANGLE_ED = lidarAngleEd;
	LIDAR_ANGLE_TOTAL = LIDAR_ANGLE_ED - LIDAR_ANGLE_ST;
	CAMERA_ANGLE_ST = cameraAngleSt;
	CAMERA_ANGLE_ED = cameraAngleEd;
	CAMERA_ANGLE_TOTAL = CAMERA_ANGLE_ED - CAMERA_ANGLE_ST;

	dataLen = 1521;//共幾筆資料
	total = CAMERA_ANGLE_TOTAL*(dataLen / LIDAR_ANGLE_TOTAL);//換算為相機後共幾筆資料
	st = (CAMERA_ANGLE_ST - LIDAR_ANGLE_ST)*(dataLen / LIDAR_ANGLE_TOTAL)-1;
	ed = st + total;	
}

vector<long> FusionManager::getLidarDistanceData()
{
	return _lidarDistanceData;
}

void FusionManager::setLidarDistanceData(vector<long> lidarDistanceData)
{
	_lidarDistanceData = lidarDistanceData;
}
