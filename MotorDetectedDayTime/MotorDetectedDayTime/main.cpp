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
	
	//frontback
	
	//string date = "1220_7";
	//string date = "1220_5";//no

	//string date = "1202_2";	
	//string date = "1213_8";//horiz needs to modify
	//string date = "1202_9";//only one motorcyclist	
	//string date = "1220_5";//no
	//string date = "1220_6";
	//string date = "1220_7";
	//string date = "1220_8";
	//string date = "1220_9";//maybe
	//string date = "1220_10";//maybe
	string date = "1202_8";//great
	//string date = "1213_2";NOT YET


	//side
	//string date = "1202_5";
	//string date = "1202_6";
	//string date = "1202_7";//great horizo 0.6
	
	//string date = "1212_1";
	//string date = "1212_2";	
	
	//string date = "1213_11";
	//string date = "1213_10";
	


	//ped
	//string date = "0116_12";
	//string date = "0116_11";
	//string date = "0116_10";
	//string date = "0116_12";
//	string date = "0116_8";
	
	
	
	string pedname = "ee_005";
	
	
	//string videopath = "Test Video\\unknow\\GH010152.MP4";	//1080x720
	//string videopath = "Test Video\\unknow\\GH010110.MP4";	//1920x1080
	
	//string videopath = "Test Video\\old\\" + pedname + ".avi";
	string videopath = "Test Video\\videoData" + date + ".avi";	
	OfflineMode *mode = new OfflineMode(videopath, FusionType::CarFront, 1 );
	mode->Run();	
	//mode->RunCompare();
	delete mode;	


	system("PAUSE");
	return 0;
}