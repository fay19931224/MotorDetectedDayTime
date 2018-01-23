#include "OfflineMode.h"
#include <sstream>
//#define liadrImformation			
//#define fpsImformation
//#define UdpSocketServer
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
		_fusionManager->SyncLidarAndCamera(-95, 95, -39, 39);
	}
	
	string headSVMXML = "Features\\人頭1219C_SVC_LINEAR.xml";
	HeadDetecter* headDetectFrontBack = new HeadDetecter(headSVMXML);
	HeadDetecter* headDetectSide = new HeadDetecter(headSVMXML);
	//svmDetectParameter headDetectParameter{ Size(32, 32),Size(8,8),static_cast<float>(0),Size(8,8),Size(8,8),1.05,2,false };	
	
	svmDetectParameter sideSvmDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.6),Size(8,8),Size(8,8),1.2,2,false };
	svmDetectParameter frontbackSvmDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.7),Size(8,8),Size(8,8),1.2,2,false };	
	svmDetectParameter car1SvmDetectParameter{ Size(64, 48),Size(8,8),static_cast<float>(1.6),Size(),Size(),1.05,2,false };
	svmDetectParameter pedestrianSvmDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.5),Size(),Size(),1.05,2,false };

	_classifierList.push_back(new SvmClassifier("Features\\正背面0123C_SVC_LINEAR.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackSvmDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier("Features\\側面0123C_SVC_LINEAR.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideSvmDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier("Features\\vehicleFeature_v1.xml", ClassiferType::CarFrontBack, Scalar(0, 0, 255), car1SvmDetectParameter, _fusionManager));
	_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianSvmDetectParameter, _fusionManager));
	
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
vector<SentData> OfflineMode::Detect(Mat &frame, Mat &grayFrame,int count)
{				
	int motobackfrontCount = 3;	
	int carbackfrontCount = 2;
	int motosideCount = 5;

	switch (0)
	{		
		case 0:			
			if (count % motobackfrontCount == 0)
			{
				((SvmClassifier*)_classifierList[0])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);				
			}
			else if (count % motosideCount == 0)
			{
				((SvmClassifier*)_classifierList[1])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);				
			}										
			else
			{
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);
			}

			if (count % carbackfrontCount == 0) 
			{
				((SvmClassifier*)_classifierList[2])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[3])->startClassify(frame, grayFrame);
			}
			else 
			{
				((SvmClassifier*)_classifierList[2])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[3])->startUpdateTrack(frame);
			}

			for (int i = 0;i< _classifierList.size(); i++) 
			{
				((SvmClassifier*)_classifierList[i])->stop();
			}			
			
			break;
		case 1:
			for (int k = 0; k < _classifierList.size(); k++)
			{
				((SvmClassifier*)_classifierList[k])->startClassify(frame, grayFrame);
				//((SvmClassifier*)_classifierList[k])->Classify(frame, grayFrame);
			}
			for (int k = 0; k < _classifierList.size(); k++)
			{
				((SvmClassifier*)_classifierList[k])->stop();
				//((SvmClassifier*)_classifierList[k])->Update_track(frame);
			}
			break;	
		case 2:
			if (count % motobackfrontCount == 0)
			{
				((SvmClassifier*)_classifierList[0])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);			
			}
			else if (count % motosideCount == 0)
			{
				((SvmClassifier*)_classifierList[1])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);			
			}			
			else
			{
				for (int k = 0; k < 2; k++)
				{
					((SvmClassifier*)_classifierList[k])->startUpdateTrack(frame);
				}
			}
			((SvmClassifier*)_classifierList[0])->stop();
			((SvmClassifier*)_classifierList[1])->stop();			
			break;
		case 3:
			((SvmClassifier*)_classifierList[2])->startClassify(frame, grayFrame);
			((SvmClassifier*)_classifierList[2])->stop();
	}
	vector<SentData> result;
#ifdef UdpSocketServer
	vector<SentData>* frontbackResult = ((SvmClassifier*)_classifierList[0])->getSentData();
	vector<SentData>* SideResult = ((SvmClassifier*)_classifierList[1])->getSentData();
	result.insert(result.end(), frontbackResult->begin(), frontbackResult->end());
	result.insert(result.end(), SideResult->begin(), SideResult->end());
#endif // UdpSocketServer
	return result;
}



/*!
* 1.使用lidar資料時，讀取影像及Lidar資料後，對影像中進行物件的偵測並顯示出偵測結果
* 2.僅使用影像資料時，對影像中進行霍夫圓偵測尋找車輪，進行ROI的推估後
* 對ROI區域進行物件的偵測並顯示出偵測結果
*/
void OfflineMode::Run()
{

#ifdef UdpSocketServer
	SocketServer _socketServer;
#endif // UdpSocketServer
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
	writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), reader->GetCameraFPS(), reader->getVideoSize());
	if (!writer.isOpened())
	{
		return;
	}	
	int fps=reader->GetCameraFPS();

	clock_t StartTime=0;
	clock_t EndTime=0;
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
		else 
		{
			lidarDistanceData.clear();
			lidarSignalData.clear();			
			((LidarReader*)reader)->RequestData(frame, lidarDistanceData, lidarSignalData,_fusionManager->getReadLidarPosition().first, _fusionManager->getReadLidarPosition().second);			
			_fusionManager->setLidarDistanceData(lidarDistanceData);
		}
	
	

		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		
		StartTime = clock();
		Rect ROI = Rect(0, frame.rows / 8, frame.cols, frame.rows * 7 / 8);
		//Rect ROI = Rect(0, frame.rows / 2, frame.cols, frame.rows /2);
		vector<SentData> SentDatavector=Detect(frame(ROI), grayFrame(ROI), i);		
		cv::rectangle(frame, ROI, Scalar(0, 0, 0), 2);		
		EndTime = clock();
		

		#ifdef UdpSocketServer
		for (int i = 0;i<SentDatavector.size();i++) 
		{			
			SentData temp = SentDatavector[i];
			_socketServer.sentData(temp.ROI_Left_Top_X,temp.ROI_Left_Top_Y,
			temp.ROI_Width,temp.ROI_Height,temp.Object_Distance,temp.Object_Type
			);
		}		
		#endif // UdpSocketServer

		#ifdef fpsImformation
		int dt = EndTime - StartTime;		
		std::stringstream ss;		
		ss << (1000 / dt);
		std::string fpsString("FPS:");
		fpsString += ss.str();
		std::stringstream ss2;
		ss2 << i;		
		putText(frame, fpsString, CvPoint(0, frame.rows - 25), 0, 1, Scalar(255, 255, 255), 1, 8, false);
		putText(frame, ss2.str(), CvPoint(0, frame.rows - 50), 0, 1, Scalar(255, 255, 255), 1, 8, false);
		std::string name = "pic\\wholeframe\\" + ss2.str() + ".jpg";
		cv::imwrite(name, frame);
		#endif // fpsImformation


		#ifdef liadrImformation							
		Mat frameWithLidar;
		cv::vconcat(frame, _fusionManager->showLidarImformation(frame), frameWithLidar);		
		//cv::imshow("distanceLida", _fusionManager->showWholeLidarImformation(frame));
		//_fusionManager->showWholeLidarImformation(frame);
		cv::imshow("distance", frameWithLidar);						
		#else
			imshow(_videoFileName, frame);			
			writer.write(frame);
		#endif			
		_waitKeySec = 30;
		if (WaitKey())
		{
			break;
		}	
	}	
	//cout <<"average fps:"<< sum/i << endl;

	#ifdef UdpSocketServer
		_socketServer.closeServer();
	#endif // UdpSocketServer	
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