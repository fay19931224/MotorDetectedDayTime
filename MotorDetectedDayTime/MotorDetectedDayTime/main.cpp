#include "OfflineMode.h"
//#include "ClassifierTestingMode.h"
#include <Windows.h>
#include <time.h>
int main()
{
	time_t nStart = time(NULL);
	Mode *mode = NULL;
	int currentModelType = 1;

	while (true)
	{
		if (currentModelType == 0)
		{
			break;
		}
		else if (currentModelType == 1)
		{
			string path = "Test Video\\0823_motorside3.mp4";
			//string path = "Test Video\\0823_carfront1.mp4";
			//string path = "Test Video\\0817_carrear.mp4";
			
			mode = new OfflineMode(path, FusionType::CarFront, currentModelType);
			//mode = new OfflineMode("Lidar Test Data\\CarRight\\videoData1216car.avi", FusionType::CarFront, currentModelType);
		}
		/*else if (currentModelType == 2)
		{
		mode = new OfflineMode("Lidar Test Data\\CarLeft\\videoData1216car.avi", "Lidar Test Data\\CarLeft\\lidarData1216car.txt", FusionType::CarLeftSide, currentModelType);
		}
		else if (currentModelType == 3)
		{
		mode = new OfflineMode("Lidar Test Data\\CarLeft\\videoData1216motor.avi", "Lidar Test Data\\CarLeft\\lidarData1216motor.txt", FusionType::CarLeftSide, currentModelType);
		}
		else if (currentModelType == 4)
		{
		mode = new OfflineMode("Lidar Test Data\\CarRight\\videoData1216car.avi", "Lidar Test Data\\CarRight\\lidarData1216car.txt", FusionType::CarRightSide, currentModelType);
		}
		else if (currentModelType == 5)
		{
		mode = new OfflineMode("Lidar Test Data\\CarRight\\videoData1216motor.avi", "Lidar Test Data\\CarRight\\lidarData1216motor.txt", FusionType::CarRightSide, currentModelType);
		}
		else if (currentModelType == 6)
		{
		mode = new OfflineMode("Lidar Test Data\\CarRear\\videoData1216.avi", "Lidar Test Data\\CarRear\\videoData1216.txt", FusionType::CarBack, currentModelType);
		}*/
		mode->Run();
		currentModelType = ((OfflineMode *)mode)->GetChoiceVideoType();
		delete mode;
		break;
	}
	time_t nEnd = time(NULL);
	cout << nEnd - nStart << endl;
	system("PAUSE");
	return 0;
}