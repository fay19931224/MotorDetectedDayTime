#include "OfflineMode.h"
#include <time.h>

int main()
{
	time_t nStart = time(NULL);	
		
	
	//string lidarpath = "";
	string videopath = "Test Video\\videoData0116_10.avi";
	string lidarpath = "Lidar Data\\lidarData0116_10.txt";
	
	//string videopath = "Test Video\\videoData1450_front.avi";
	//string videopath = "Test Video\\videoData1220_9.avi";
	//string lidarpath = "Lidar Data\\lidarData1220_9.txt";
	

	Mode *mode = new OfflineMode(videopath, lidarpath, FusionType::CarFront, 1 );
	mode->Run();

	delete mode;	
	time_t nEnd = time(NULL);
	cout << nEnd - nStart << endl;

	system("PAUSE");
	return 0;
}