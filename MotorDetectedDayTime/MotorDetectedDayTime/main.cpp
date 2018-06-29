#include "OfflineMode.h"

int main()
{	
	//cv::setUseOptimized(true);
	if (cv::useOptimized())
	{
		cout << "OpenCV optimized CPU open" << endl;
	}
	if (cv::checkHardwareSupport(CV_SSE2))
	{
		cout << "OpenCV open CV_SSE2" << endl;
	}
	else if (cv::checkHardwareSupport(CV_NEON))
	{
		cout << "OpenCV open CV_NEON" << endl;
	}
	
	//string date = "1220_2";
	//string date = "1213_11";
	string date = "0116_10";
	//string date = "1202_2";
	//string date = "1213_8";
	//string date = "1202_3";
	//string date = "1202_5";
	//string date = "1202_6";
	//string date = "1202_7";//great
	//string date = "1202_8";//great
	//string date = "1202_9";
	//string date = "1212_1";
	//string date = "1212_2";	
	//string date = "1213_2";
	//string date = "1220_6";
	//string date = "1220_9";
	//string date = "1220_10";
	
	
//	string videopath = "Test Video\\old\\2016_0302_151153_005.wmv";
	string videopath = "Test Video\\videoData" + date + ".avi";	
	OfflineMode *mode = new OfflineMode(videopath, FusionType::CarFront, 1 );
	mode->Run();	
	delete mode;

	system("PAUSE");
	return 0;
}