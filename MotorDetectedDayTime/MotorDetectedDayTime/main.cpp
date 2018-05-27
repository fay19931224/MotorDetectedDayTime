#include "OfflineMode.h"
#include <time.h>

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
	if (cv::checkHardwareSupport(CV_NEON))
	{
		cout << "OpenCV open CV_NEON" << endl;
	}
	
	string date = "1220_2";
	//string date = "1213_11";
	//string date = "0116_10";
	//string date = "1202_2";
	//string date = "1213_8";
	
	//string videopath = "Demo\\videoData" + date + ".avi";	
	//string lidarpath = "Demo\\lidarData" + date + ".txt";

	string videopath = "Test Video\\videoData" + date + ".avi";
	string lidarpath = "";

	Mode *mode = new OfflineMode(videopath, lidarpath, FusionType::CarFront, 1 );
	mode->Run();

	delete mode;		
	system("PAUSE");
	return 0;
}