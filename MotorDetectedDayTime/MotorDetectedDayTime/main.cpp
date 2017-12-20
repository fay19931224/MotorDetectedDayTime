#include "OfflineMode.h"
#include <time.h>

int main()
{
	time_t nStart = time(NULL);	
	//string path = "Test Video\\1117�s�����v��1.MOV";
	//string path = "Test Video\\1117�s�����v��4.MOV";
	//string path = "Test Video\\1117�s�����v��3.MOV";	
	//string path = "Test Video\\videoplayback08.mp4";
	//string path = "Test Video\\0823_motorside1.mp4";
	//string path = "Test Video\\0823_motorside2.mp4";
	//string path = "Test Video\\0823_motorside3.mp4";
	//string path = "Test Video\\0823_motorside4.mp4";
	//string path = "Test Video\\0823_carfront1.mp4";
	//string path = "Test Video\\0823_carfront2.mp4";	
	//string path = "Test Video\\0817_carrear.mp4";
	//string path = "Test Video\\0817_carrear2.mp4";
	//string path = "Test Video\\�榡�u�t�v��3�ŵ���37��.mp4";
	//string path = "Test Video\\�v��1�ŵ���33��.mp4";
	//string lidarpath = "";
	string videopath = "Test Video\\videoData1213_5.avi";
	string lidarpath = "Lidar Data\\lidarData1213_5.txt";
	

	Mode *mode = new OfflineMode(videopath, lidarpath, FusionType::CarFront, 1 );
	mode->Run();

	delete mode;	
	time_t nEnd = time(NULL);
	cout << nEnd - nStart << endl;

	system("PAUSE");
	return 0;
}