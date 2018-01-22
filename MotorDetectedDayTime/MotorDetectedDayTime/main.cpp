#include "OfflineMode.h"
#include <time.h>

int main()
{
	time_t nStart = time(NULL);	
		
	
	//string lidarpath = "";
	string date = "1213_5";
	string videopath = "Test Video\\videoData" + date + ".avi";
	string lidarpath = "Lidar Data\\lidarData" + date + ".txt";
	

	Mode *mode = new OfflineMode(videopath, lidarpath, FusionType::CarFront, 1 );
	mode->Run();

	delete mode;	
	time_t nEnd = time(NULL);
	cout << nEnd - nStart << endl;

	system("PAUSE");
	return 0;
}