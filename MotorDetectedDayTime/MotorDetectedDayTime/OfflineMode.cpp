#include "OfflineMode.h"
#include <sstream>

//#define liadrImformation			
#define fpsImformation
//#define SaveFrame

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
	_waitKeySec = 1;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;
	_lidarFileName = lidarFileName;
	motobackfrontFlag = true;

	if (_lidarFileName != "") 
	{
		_fusionManager = new FusionManager();
		_fusionManager->SyncLidarAndCamera(-95, 95, -39, 39);
	}
	
	
	HeadDetecter* headDetectFrontBack = new HeadDetecter();	
		
	HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(8,8),1.2,2,false ,HOGDescriptor::L2Hys };
	HogParameter frontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(8,8),1.2,2,HOGDescriptor::L2Hys };
	HogParameter pedestrianDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.5),Size(),Size(),1.05,2,HOGDescriptor::L2Hys };
	
	
	string svmModel_Path = "C:\\Users\\fay\\Source\\Repos\\HOGfeature_traing\\HOGfeature_traing\\HOGfeature_traing";	
		
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\正背面0518_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\側面0518_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, _fusionManager, headDetectFrontBack));

	/*_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\正背面0525_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\側面0525_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, _fusionManager, headDetectFrontBack));*/
	_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianDetectParameter, _fusionManager));
	
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
	
	switch (2)
	{			
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
			if (motobackfrontFlag == true)
			{
				((SvmClassifier*)_classifierList[0])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[2])->startUpdateTrack(frame);
				motobackfrontFlag = false;
				motosideCountFlag = true;
			}
			else if (motosideCountFlag == true)
			{
				((SvmClassifier*)_classifierList[1])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[2])->startUpdateTrack(frame);
				motosideCountFlag = false;
				pedfrontCountFlag = true;
			}
			else if (pedfrontCountFlag == true)
			{
				((SvmClassifier*)_classifierList[2])->startClassify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);
				pedfrontCountFlag = false;
				motobackfrontFlag = true;
			}
			else
			{
				throw exception("Detected Object Flag Error");
			}
			for (int i = 0; i< _classifierList.size(); i++)
			{
				((SvmClassifier*)_classifierList[i])->stop();
			}
			break;
		case 3:						
			((SvmClassifier*)_classifierList[1])->ClassifyTest(frame, grayFrame);
			/*for (int k = 0; k < _classifierList.size(); k++)
			{
				((SvmClassifier*)_classifierList[k])->ClassifyTest(frame, grayFrame);
			}*/
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
	
	/*VideoWriter writer;
	writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), reader->GetCameraFPS(), reader->getVideoSize());
	if (!writer.isOpened())
	{
		return;
	}	*/
	
	int i = 0;
	for (; i < dataQuantity; i++)
	{			
		Mat frame;
		Mat grayFrame;

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

		
		double t = (double)cv::getTickCount();
		/*Rect ROI = Rect(0, frame.rows / 8, frame.cols, frame.rows * 7 / 8);		
		Detect(frame(ROI), grayFrame(ROI), i);*/
		Detect(frame, grayFrame, i);
		t = (double)cv::getTickCount() - t;
		
		//cout << t*1000 << endl;
		#ifdef fpsImformation
		float dt = cv::getTickFrequency() / t;
		std::stringstream ss;		
		ss << dt;
		std::string fpsString("FPS:");
		fpsString += ss.str();
		std::stringstream ss2;
		ss2 << i;				
		putText(frame, fpsString, CvPoint(0, frame.rows - 25), 0, 1, Scalar(255, 255, 255), 1, 8, false);
		putText(frame, ss2.str(), CvPoint(0, frame.rows - 50), 0, 1, Scalar(255, 255, 255), 1, 8, false);
		
		//cout << "frame:" << i << endl << "FPS:" << dt << endl;
		#endif // fpsImformation

		#ifdef SaveFrame
		std::string name = "pic\\wholeframe\\" + ss2.str() + ".jpg";
		cv::imwrite(name, frame);
		#endif // SaveFrame

		
		imshow(_videoFileName, frame);					
		if (WaitKey())
		{
			break;
		}	
	}		
	destroyAllWindows();
}
