#include "OfflineMode.h"
#include <time.h>

int main()
{
	time_t nStart = time(NULL);			
		
	//string date = "1220_2";
	string date = "1213_11";
	//string date = "0116_10";
	//string date = "0116_6";
	
	/*string videopath = "Test Video\\videoData" + date + ".avi";
	string lidarpath = "Lidar Data\\lidarData" + date + ".txt";*/

	string videopath = "Demo\\videoData" + date + ".avi";
	string lidarpath = "Demo\\lidarData" + date + ".txt";

	Mode *mode = new OfflineMode(videopath, lidarpath, FusionType::CarFront, 1 );
	mode->Run();

	delete mode;	
	time_t nEnd = time(NULL);
	cout << nEnd - nStart << endl;

	system("PAUSE");
	return 0;
}