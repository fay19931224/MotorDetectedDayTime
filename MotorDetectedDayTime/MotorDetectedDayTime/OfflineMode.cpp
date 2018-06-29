#include "OfflineMode.h"
#include <sstream>
		
#define fpsImformation
//#define SaveFrame

OfflineMode::OfflineMode(string videoFileName, FusionType type, int currentModelType = 0)
{	
	_type = type;
	_waitKeySec = 1;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;	
	motobackfrontFlag = true;
	
	HeadDetecter* headDetectFrontBack = new HeadDetecter();			
	
	HogParameter frontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.7),Size(8,8),Size(),1.2,2,false,HOGDescriptor::L2Hys,0.5 };
	HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(),1.15,2,false ,HOGDescriptor::L2Hys,0.5 };
	HogParameter pedestrianDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.8),Size(),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5 };
	HogParameter IRpedestrianDetectParameter{ Size(64, 128),Size(8,8),static_cast<float>(1),Size(),Size(),1.15,2,false,HOGDescriptor::L2Hys,0.5 };
	//HogParameter IRpedestrianDetectParameter{ Size(64, 128),Size(8,8),static_cast<float>(1),Size(),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5 };
		
	string svmModel_Path = "C:\\Users\\fay\\Source\\Repos\\HOGfeature_traing\\HOGfeature_traing\\HOGfeature_traing";	
		
	

	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\frontback0529_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\side0529_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, headDetectFrontBack));	
	_classifierList.push_back(new SvmClassifier("Features\\pedestrianFeature.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), pedestrianDetectParameter));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\ped0626_auto.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), IRpedestrianDetectParameter));
	
	
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

void OfflineMode::Detect(Mat &frame, Mat &grayFrame,int count)
{					
	int i = 0;
	switch (0)
	{	
		case 0:			
			//((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
			//((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);			
			((SvmClassifier*)_classifierList[3])->ClassifyPedes(frame, grayFrame);
			break;
		case 1:
			if (motobackfrontFlag == true)
			{			
				
				((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
				((SvmClassifier*)_classifierList[1])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[2])->startUpdateTrack(frame);
				
				motobackfrontFlag = false;
				motosideCountFlag = true;
			}
			else if (motosideCountFlag == true)
			{				
				((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->startUpdateTrack(frame);
				((SvmClassifier*)_classifierList[2])->startUpdateTrack(frame);
				motosideCountFlag = false;
				pedfrontCountFlag = true;
			}
			else if (pedfrontCountFlag == true)
			{			
				((SvmClassifier*)_classifierList[2])->Classify(frame, grayFrame);
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
		case 2:	
			if (mode ==0) 
			{
				((SvmClassifier*)_classifierList[i])->Classify(frame, grayFrame);
				mode = 1;
			}
			else if (mode == 1) 
			{
				if (i == 2) 
				{
					((SvmClassifier*)_classifierList[2])->Update_track(frame);
				}
				else 
				{
					((SvmClassifier*)_classifierList[i])->Update_trackPedes(frame);
				}
				mode = 0;
			}			
			break;
		case 3:
			if (motobackfrontFlag == true)
			{
				//unsigned long start = clock();
				//((SvmClassifier*)_classifierList[0])->newClassify(frame, grayFrame);//0.030~0.060
				//unsigned long end = clock();
				//printf("time=%1.3f seconds\n", (end - start) / 1000.0);
				//putText(frame, "frontback", CvPoint(0, 20), 0, 1, Scalar(255, 255, 255), 1, 8, false);				
				((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);//0.030~0.060
				((SvmClassifier*)_classifierList[1])->Update_track(frame);			//0.007~0.010	
				((SvmClassifier*)_classifierList[2])->Update_trackPedes(frame);		//0.007~0.010			
				
				motobackfrontFlag = false;
				motosideCountFlag = true;
			}
			else if (motosideCountFlag == true)
			{
				//putText(frame, "side", CvPoint(0, 20), 0, 1, Scalar(255, 255, 255), 1, 8, false);				
				((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->Update_track(frame);				
				((SvmClassifier*)_classifierList[2])->Update_trackPedes(frame);			
				
				motosideCountFlag = false;
				pedfrontCountFlag = true;
			}
			else if (pedfrontCountFlag == true)
			{
				//putText(frame, "Ped", CvPoint(0, 20), 0, 1, Scalar(255, 255, 255), 1, 8, false);				
				((SvmClassifier*)_classifierList[2])->ClassifyPedes(frame, grayFrame);
				((SvmClassifier*)_classifierList[0])->Update_track(frame);
				((SvmClassifier*)_classifierList[1])->Update_track(frame);
				
				pedfrontCountFlag = false;
				motobackfrontFlag = true;
			}
			else
			{
				throw exception("Detected Object Flag Error");
			}			
			break;
	}	
}
inline Rect test(cv::Mat frame) {
	int detectHeight = 144;
	float scale = 0.5;

	int x = 0;
	int y = (frame.rows*scale - detectHeight) > 0 ? (frame.rows*scale - detectHeight) : 0;
	int width = frame.cols;
	int height= (frame.rows*scale -y)+ (frame.rows*scale+detectHeight>frame.rows ? frame.rows-frame.rows*scale: detectHeight);
	cout << "frame's width:" <<  frame.cols<< endl;
	cout << "frame's height:" << frame.rows << endl;

	cout << "x:" << x << endl;
	cout << "y:" << y << endl;
	cout << "width:" << width << endl;
	cout << "height:" << height << endl << endl;	
	return Rect(x, y, width, height);
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
	

	for (int i = 0; i < dataQuantity; i++)
	{			
		Mat frame;
		Mat grayFrame;		
		reader->RequestData(frame);
		//resize(frame, frame, Size(), 0.5, 0.5);

		/*_waitKeySec = 0;
		Mat dst;
		std::vector<cv::Size> size;
		size.push_back(Size(640,480));
		size.push_back(Size(533, 400));
		size.push_back(Size(444, 333));
		size.push_back(Size(370, 278));
		size.push_back(Size(309,231));
		size.push_back(Size(257,193));
		size.push_back(Size(214,161));
		size.push_back(Size(179,134));
		size.push_back(Size(149,112));

		for (int j = 0;j<size.size(); j++)
		{
			resize(frame, dst,size[j]);
			stringstream s;
			s << j;
			imshow(_videoFileName + s.str(), dst(test(dst)));
		}*/

		cvtColor(frame, grayFrame, CV_BGR2GRAY);
		
		double t = (double)cv::getTickCount();		
		//unsigned long start = clock();
		Detect(frame, grayFrame, i);			
		//unsigned long end = clock();
		t = (double)cv::getTickCount() - t;
		
		//printf("%d time=%1.3f seconds\n", i % 3, (end - start) / 1000.0);

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
		std::string name = "result\\" + ss2.str() + ".jpg";
		cv::imwrite(name, frame);
		#endif // SaveFrame

		
		imshow(_videoFileName, frame);	
		
		waitKey(_waitKeySec);
			
	}		
	destroyAllWindows();
}


