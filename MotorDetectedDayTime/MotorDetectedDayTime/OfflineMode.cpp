#include "OfflineMode.h"
#include <sstream>

/*!
* 取得影像，雷達名稱以及fusion的類型
* 根據fusion類型決定ROI距離以及起始偵測距離，將Lidar資料與影像資料進行校正
* 根據分類器類型載入對應的Model放進classifierList供偵測使用
* @param videlFileName 為string 類型，為要偵測的影片名稱
* @param lidarFileName 為string 類型，為要偵測的影片對應的Lidar資料名稱
* @param videlFileName 為FusionType 類型，為fusion的類型
*/
OfflineMode::OfflineMode(string videoFileName, string lidarFileName, FusionType type, int currentModelType = 0)
{
	_type = type;
	_waitKeySec = 30;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;
	_lidarFileName = lidarFileName;

	if (_lidarFileName != "") 
	{
		_fusionManager = new FusionManager();
		_fusionManager->InititalizeDistanceLimit(8000, 15000);
		_fusionManager->SyncLidarAndCamera(-95, 95, -39, 39);
	}
	
	svmDetectParameter sideSvmDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(8,8),1.2,2,false };
	svmDetectParameter frontbackSvmDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.7),Size(8,8),Size(8,8),1.2,2,false };
	svmDetectParameter headDetectParameter{ Size(32, 32),Size(8,8),static_cast<float>(0),Size(8,8),Size(8,8),1.05,2,false };

	//SvmClassifier* headdetectd=new SvmClassifier("Features\\head.xml", ClassiferType::Helmet, Scalar(0, 0, 255), headDetectParameter);
	HeadSVMDetecter* headSVMDetectFrontBack = new HeadSVMDetecter("Features\\head.xml");
	HeadSVMDetecter* headSVMDetectSide = new HeadSVMDetecter("Features\\head.xml");

	_classifierList.push_back(new SvmClassifier("Features\\FrontBackReflect_C_SVC_LINEAR.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackSvmDetectParameter, headSVMDetectFrontBack, _fusionManager));
	_classifierList.push_back(new SvmClassifier("Features\\SideReflect_C_SVC_LINEAR.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideSvmDetectParameter, headSVMDetectSide, _fusionManager));
}

OfflineMode::~OfflineMode()
{
}

bool OfflineMode::WaitKey()
{
	int currentType = waitKey(_waitKeySec);
	if ('1' <= currentType && currentType <= '6')
	{
		_waitKeyChoosen = currentType - '0';
		return true;
	}
	else if (currentType == 27)//ESC
	{
		_waitKeyChoosen = 0;
		return true;
	}
	return false;
}

/*!
* 對ROI進行調整，將超出邊框的部分設定回邊界
* @param frame 為Mat型態，為輸入的原始影像
* @param roi 為Rect型態，為要進行物件偵測的區域
*/
Rect OfflineMode::adjustROI(Mat frame, Rect roi)
{
	if (roi.x<0)
	{
		roi.x = 0;
	}
	if (roi.y<0)
	{
		roi.y = 0;
	}
	if (roi.x + roi.width>frame.cols)
	{
		roi.width = frame.cols - roi.x;
	}
	if (roi.y + roi.height> frame.rows)
	{
		roi.height = frame.rows - roi.y;
	}
	return roi;
}


/*!
* @param frame 為Mat型態，為輸入的原始影像
* @param grayFrame 為Mat型態，原始影像灰階後的影像
*/
void OfflineMode::Detect(Mat &frame, Mat &grayFrame,int count)
{				
	int backfrontCount = 3;	
	int sideCount = 5;

	switch (0)
	{
		case 0:
			if (count % backfrontCount == 0)
			{
				((SvmClassifier*)_classifierList[0])->start(frame, grayFrame);				
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);
			}
			else if (count % sideCount == 0)
			{
				((SvmClassifier*)_classifierList[1])->start(frame, grayFrame);				
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);
			}							
			else
			{
				for (int k = 0; k < _classifierList.size(); k++)
				{				
					((SvmClassifier*)_classifierList[k])->startUpdateTrack(frame);
				}
			}
			((SvmClassifier*)_classifierList[0])->stop();
			((SvmClassifier*)_classifierList[1])->stop();
			break;
		case 1:
			for (int k = 0; k < _classifierList.size(); k++)
			{
				//((SvmClassifier*)_classifierList[k])->start(frame, grayFrame);
				((SvmClassifier*)_classifierList[k])->Classify(frame, grayFrame);
			}
			for (int k = 0; k < _classifierList.size(); k++)
			{
				//((SvmClassifier*)_classifierList[k])->stop();
				((SvmClassifier*)_classifierList[k])->Update_track(frame);
			}
			break;	
	}
		

}



/*!
* 1.使用lidar資料時，讀取影像及Lidar資料後，對影像中進行物件的偵測並顯示出偵測結果
* 2.僅使用影像資料時，對影像中進行霍夫圓偵測尋找車輪，進行ROI的推估後
* 對ROI區域進行物件的偵測並顯示出偵測結果
*/
void OfflineMode::Run()
{
	VideoReader* reader;
	if (_lidarFileName == "") 
	{
		reader = new VideoReader(_videoFileName);
	}
	else {
		reader = new LidarReader(_videoFileName, _lidarFileName);
	}
	
	reader->StartRead();
	int dataQuantity = reader->GetDataQuantity();
	
	VideoWriter writer;
	writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(640, 360));
	if (!writer.isOpened())
	{
		return;
	}	
	int fps=reader->GetCameraFPS();

	clock_t StartTime;
	clock_t EndTime;
	int sum = 0;
	int i = 0;
	for (; i < dataQuantity; i++)
	{			
		Mat frame;
		Mat grayFrame;
		
		//if (frame.rows > 360)
		//{
		//	resize(frame, frame, Size(640, 360), CV_INTER_LINEAR);
		//	//resize(frame, frame, Size(0, 0),0.5,0.5, CV_INTER_LINEAR);
		//}

		if (_lidarFileName == "")
		{
			reader->RequestData(frame);
		}
		else {
			lidarDistanceData.clear();
			lidarSignalData.clear();
			((LidarReader*)reader)->RequestData(frame, lidarDistanceData, lidarSignalData);
			_fusionManager->setLidarDistanceData(lidarDistanceData);
		}
				
		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		StartTime = clock();
		Detect(frame, grayFrame, i);
		EndTime = clock();
		int dt = EndTime - StartTime;
		
		sum += (1000.0 / dt);
		std::stringstream ss;
		ss << (1000.0 / dt);
		std::string fpsString("FPS:");
		fpsString += ss.str().substr(0,4);		
		std::stringstream ss2;
		ss2 << i;

		putText(frame, "Green:Front", CvPoint(0, 25), 0, 1, Scalar(0, 255, 0), 1, 8, false);
		putText(frame, "Blue:Side", CvPoint(0, 50), 0, 1, Scalar(255, 0, 0), 1, 8, false);
		putText(frame, fpsString, CvPoint(0, frame.rows - 25), 0, 1, Scalar(255, 255, 255), 1, 8, false);
		putText(frame, ss2.str(), CvPoint(0, frame.rows - 50), 0, 1, Scalar(255, 255, 255), 1, 8, false);

		
		imshow(_videoFileName, frame);
		
		std::string name = "pic\\wholeframe\\" + ss2.str() + ".jpg";		
		cv::imwrite(name, frame);
		//writer.write(frame);		
		//cvWaitKey(0);		
		if (WaitKey())
		{
			break;
		}
		
	}
	//cout <<"average fps:"<< sum/i << endl;
	destroyAllWindows();
}
//void OfflineMode::OnGrab(void *info)
//{
//	// 在影像一進來的時後加入計算 Frame Rate
//	static clock_t       StartTime = clock();
//
//
//	clock_t                 EndTime = clock();
//	int                        dt = EndTime - StartTime;
//	StartTime = EndTime;
//	if (dt != 0)
//	{
//		cout << "Frame : " << 1000.0 / dt << endl;
//	}
//}