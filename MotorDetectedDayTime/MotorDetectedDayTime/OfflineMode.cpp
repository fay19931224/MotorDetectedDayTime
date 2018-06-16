#include "OfflineMode.h"
#include <sstream>

//#define liadrImformation			
#define fpsImformation
//#define SaveFrame

OfflineMode::OfflineMode(string videoFileName, string lidarFileName, FusionType type, int currentModelType = 0)
{	
	_type = type;
	_waitKeySec = 1;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;
	_lidarFileName = lidarFileName;
	motobackfrontFlag = true;
	
	HeadDetecter* headDetectFrontBack = new HeadDetecter();			
	
	HogParameter frontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(),1.2,2,HOGDescriptor::L2Hys };
	HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.3),Size(8,8),Size(),1.2,2,false ,HOGDescriptor::L2Hys };
	HogParameter pedestrianDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.8),Size(),Size(),1.05,2,HOGDescriptor::L2Hys };
		
	string svmModel_Path = "C:\\Users\\fay\\Source\\Repos\\HOGfeature_traing\\HOGfeature_traing\\HOGfeature_traing";	
		
	/*_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\正背面0518_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\側面0518_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, _fusionManager, headDetectFrontBack));*/

	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\正背面0529_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\側面0529_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, _fusionManager, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianDetectParameter, _fusionManager));
	//_classifierList.push_back(new SvmClassifier("Features\\彰人.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianDetectParameter, _fusionManager));
	
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
* @param frame 為Mat型態，為輸入的原始影像
* @param grayFrame 為Mat型態，原始影像灰階後的影像
*/
void OfflineMode::Detect(Mat &frame, Mat &grayFrame,int count)
{				
	
	switch (3)
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
			//((SvmClassifier*)_classifierList[1])->ClassifyTest(frame, grayFrame);//FPS有時候30
			((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);//FPS快20

			//跑ClassifyTest時20上下
			//跑Classify時15上下
			/*((SvmClassifier*)_classifierList[1])->startClassify(frame, grayFrame);
			((SvmClassifier*)_classifierList[1])->stop();*/
			
			break;
		case 4:
			if (motobackfrontFlag == true)
			{
				((SvmClassifier*)_classifierList[1])->Update_track(frame);
				((SvmClassifier*)_classifierList[2])->Update_trackPedes(frame);
				((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
				motobackfrontFlag = false;
				motosideCountFlag = true;
			}
			else if (motosideCountFlag == true)
			{
				((SvmClassifier*)_classifierList[0])->Update_track(frame);
				((SvmClassifier*)_classifierList[2])->Update_trackPedes(frame);
				((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);
				motosideCountFlag = false;
				pedfrontCountFlag = true;
			}
			else if (pedfrontCountFlag == true)
			{				
				((SvmClassifier*)_classifierList[0])->Update_track(frame);
				((SvmClassifier*)_classifierList[1])->Update_track(frame);
				((SvmClassifier*)_classifierList[2])->ClassifyPedes(frame, grayFrame);
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
	}	
}

void OfflineMode::Run()
{	
	VideoReader* reader;
	reader = new VideoReader(_videoFileName);
	cout << reader->StartRead() << endl;
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

		reader->RequestData(frame);
		
		cvtColor(frame, grayFrame, CV_BGR2GRAY);

		double t = (double)cv::getTickCount();
		/*Rect ROI = Rect(0, frame.rows / 8, frame.cols, frame.rows * 7 / 8);		
		Detect(frame(ROI), grayFrame(ROI), i);*/
		Detect(frame, grayFrame, i);
		t = (double)cv::getTickCount() - t;
		
		#ifdef fpsImformation
		int dt =int( cv::getTickFrequency() / t);
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

void OfflineMode::RunThread()
{
	HeadDetecter* headDetectFrontBack = new HeadDetecter();

	HogParameter frontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.2,2,HOGDescriptor::L2Hys };
	HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.2,2,false ,HOGDescriptor::L2Hys };
	HogParameter pedestrianDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.5),Size(),Size(),1.05,2,HOGDescriptor::L2Hys };
	
	string svmModel_Path = "C:\\Users\\fay\\Source\\Repos\\HOGfeature_traing\\HOGfeature_traing\\HOGfeature_traing";	

	VideoReader* reader;
	reader = new VideoReader(_videoFileName);
	cout << reader->StartRead() << endl;

	SvmClassifier* frontBackMotorClassifier =new SvmClassifier(svmModel_Path + "\\正背面0529_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, headDetectFrontBack, reader);
	SvmClassifier* sideMotorClassifier = new SvmClassifier(svmModel_Path + "\\側面0529_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, headDetectFrontBack, reader);
	SvmClassifier* pedestrianClassifier = new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianDetectParameter, reader);
	
	
	bool mainFlag = false;
	condition_variable cond;

	frontBackMotorClassifier->_mainFlag = &mainFlag;
	frontBackMotorClassifier->_cond = &cond;

	sideMotorClassifier->_mainFlag = &mainFlag;
	sideMotorClassifier->_cond = &cond;
			
	pedestrianClassifier->_mainFlag = &mainFlag;
	pedestrianClassifier->_cond = &cond;

	void (SvmClassifier::*myFunc)();
	myFunc = &SvmClassifier::useThread;

	mutex mutex;
	std::unique_lock <std::mutex> lck(mutex);
	/*std::unique_lock <std::mutex> frontBacklck(frontBackMotorClassifier->_mutex);
	std::unique_lock <std::mutex> sideMotorlck(sideMotorClassifier->_mutex);
	std::unique_lock <std::mutex> pedeslck(pedestrianClassifier->_mutex);*/
	

	thread* t1 = new thread(&SvmClassifier::useThread, frontBackMotorClassifier);
	thread* t2 = new thread(&SvmClassifier::useThread, sideMotorClassifier);
	thread* t3 = new thread(&SvmClassifier::useThread, pedestrianClassifier);
	

	
	for (int i = 0; i < reader->GetDataQuantity(); i++)
	{				
		
		reader->setFrame();
		frame = reader->getFrame();
		while (!frontBackMotorClassifier->_mainStartFlag || !frontBackMotorClassifier->_mainStartFlag || !pedestrianClassifier->_mainStartFlag)
		{
			//printf("waitChildarrival\n");
		}
		printf("mainSrart_%d--------------\n", i);
		mainFlag = true;
		cond.notify_all();

		while (!frontBackMotorClassifier->_start || !frontBackMotorClassifier->_start || !pedestrianClassifier->_start)
		{
			printf("chlid not start\n");
		}

		mainFlag = false;
		frontBackMotorClassifier->_mainStartFlag = false;
		frontBackMotorClassifier->_mainStartFlag = false;
		pedestrianClassifier->_mainStartFlag = false;
		//這邊太快，使其他執行續往下跑將_mainStartFlag設為t，於是就一直waitsetting
		while (!frontBackMotorClassifier->_childDone || !frontBackMotorClassifier->_childDone || !pedestrianClassifier->_childDone)
		{
			printf("waitChildDone\n");
		}

		/*frontBackMotorClassifier->drawObject(frame);
		sideMotorClassifier->drawObject(frame);
		pedestrianClassifier->drawObject(frame);
*/
		printf("mainEnd_%d--------------\n", i);
		imshow(_videoFileName, frame); 

		if (WaitKey())
		{
			break;
		}		
	}
	printf("done");
	system("PAUSE");
	destroyAllWindows();
	
}
