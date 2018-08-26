#include "OfflineMode.h"
#include <sstream>
		
#define fpsImformation

OfflineMode::OfflineMode(string videoFileName, FusionType type, int currentModelType = 0)
{	
	_type = type;
	_waitKeySec = 1;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;	
	pedfrontCountFlag = true;
	
	string svmModel_Path = "C:\\Users\\fay\\Source\\Repos\\HOGfeature_traing\\HOGfeature_traing\\HOGfeature_traing";

	//HeadDetecter* headDetector= new HeadDetecter();		

	//string headSVMfile="\\helmet0818_auto.xml";
	string headSVMfile = "\\helmet0822_auto.xml";
	HogParameter headParameter{ Size(32, 32),Size(2,2),static_cast<float>(0.8),Size(1,1),Size(),1.1,2,false,HOGDescriptor::L2Hys,-1 ,false};
	HeadDetecter* headDetector = new HeadDetecter(svmModel_Path+headSVMfile,headParameter);
	
	HogParameter frontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.8),Size(8,8),Size(),1.15,2,false,HOGDescriptor::L2Hys,0.5 ,true};	
	HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.5),Size(8,8),Size(),1.2,2,false ,HOGDescriptor::L2Hys,0.6,true };
	//HogParameter sideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.05,2,false ,HOGDescriptor::L2Hys,-1,true };
	//HogParameter pedestrianDetectParameter{ Size(64, 144),Size(8,8),static_cast<float>(1.8),Size(),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5,true };
	
	HogParameter IRpedestrianDetectParameter{ Size(64, 128),Size(8,8),static_cast<float>(1),Size(),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5 ,true };
	
	
			
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\frontback0529_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), frontbackDetectParameter, headDetector));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\side0703_auto.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), sideDetectParameter, headDetector));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\ped0626_auto.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), IRpedestrianDetectParameter));
	
	//原版
	HogParameter orgfrontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.2,2,false,HOGDescriptor::L2Hys,0.5 ,false};
	HogParameter orgsideDetectParameter{ Size(72, 88),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.1,2,false ,HOGDescriptor::L2Hys,0.5,false };
	HogParameter orgIRpedestrianDetectParameter{ Size(64, 128),Size(8,8),static_cast<float>(1),Size(),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5,false };
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\frontback0721_autoOrig.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), orgfrontbackDetectParameter, headDetector));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\side0721_autoOrig.xml", ClassiferType::MotorbikeSide, Scalar(255, 0, 0), orgsideDetectParameter, headDetector));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\ped0721_autoOrig.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), orgIRpedestrianDetectParameter));



	/*HogParameter testfrontbackDetectParameter{ Size(48, 104),Size(8,8),static_cast<float>(0.0),Size(8,8),Size(),1.05,2,false,HOGDescriptor::L2Hys,0.5 };
	HogParameter testIRpedestrianDetectParameter{ Size(64, 128),Size(8,8),static_cast<float>(0.4),Size(),Size(),1.1,2,false,HOGDescriptor::L2Hys,0.5 };
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\moto0717_auto.xml", ClassiferType::MotorbikeFrontBack, Scalar(0, 255, 0), testfrontbackDetectParameter, headDetectFrontBack));
	_classifierList.push_back(new SvmClassifier(svmModel_Path + "\\pe0717_auto.xml", ClassiferType::Pedestrian, Scalar(0, 255, 255), testIRpedestrianDetectParameter));*/
	
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
			//object=((SvmClassifier*)_classifierList[1])->ClassifyWithHelmet(frame, grayFrame);
			//((SvmClassifier*)_classifierList[1])->ClassifyTest(frame, grayFrame);
			//((SvmClassifier*)_classifierList[0])->ClassifyTest(frame, grayFrame);
			//((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
			//object=((SvmClassifier*)_classifierList[3])->Classify(frame, grayFrame);
			//object = ((SvmClassifier*)_classifierList[0])->ClassifyTest(frame, grayFrame);
			//object = ((SvmClassifier*)_classifierList[2])->ClassifyPedes(frame, grayFrame);		
			//frontbackobject = ((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);//new
			//frontbackobject = ((SvmClassifier*)_classifierList[0])->ClassifyTest(frame, grayFrame);//new

			//frontbackobject = ((SvmClassifier*)_classifierList[3])->Classify(frame, grayFrame);//org
			//frontbackobject = ((SvmClassifier*)_classifierList[3])->ClassifyTest(frame, grayFrame);//org

			//frontbackobject = ((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
			//sideobject= ((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);
			pedobject = ((SvmClassifier*)_classifierList[2])->ClassifyPedes(frame, grayFrame);
			break;
		case 1:
			if (motobackfrontFlag == true)
			{							
				frontbackobject=((SvmClassifier*)_classifierList[0])->Classify(frame, grayFrame);
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
			
			for (int i = 0; i< _classifierList.size(); i++)
			{
				((SvmClassifier*)_classifierList[i])->stop();				
			}
			break;
		case 3:			
			if (motosideCountFlag == true)
			{
				//putText(frame, "side", CvPoint(0, 20), 0, 1, Scalar(255, 255, 255), 1, 8, false);				
				sideobject = ((SvmClassifier*)_classifierList[1])->Classify(frame, grayFrame);
				vector<Rect> object;
				((SvmClassifier*)_classifierList[2])->Update_trackPedes(frame, object);
				
				motosideCountFlag = false;
				pedfrontCountFlag = true;
			}
			else if (pedfrontCountFlag == true)
			{
				//putText(frame, "Ped", CvPoint(0, 20), 0, 1, Scalar(255, 255, 255), 1, 8, false);				
				pedobject=((SvmClassifier*)_classifierList[2])->ClassifyPedes(frame, grayFrame);
				((SvmClassifier*)_classifierList[1])->Update_track(frame);
				
				pedfrontCountFlag = false;
				motosideCountFlag = true;
			}					
			break;
	}	
}
inline Rect test(cv::Mat frame) {
	int detectHeight = 104;
	float scale = 0.5;

	int x = 0;
	int y = (frame.rows*scale - detectHeight) > 0 ? (frame.rows*scale - detectHeight) : 0;
	int width = frame.cols;
	int height= (frame.rows*scale -y)+ (frame.rows*scale+detectHeight>frame.rows ? frame.rows-frame.rows*scale: detectHeight);
	/*cout << "frame's width:" <<  frame.cols<< endl;
	cout << "frame's height:" << frame.rows << endl;

	cout << "x:" << x << endl;
	cout << "y:" << y << endl;
	cout << "width:" << width << endl;
	cout << "height:" << height << endl;	*/
	cout << "area:" << width*height << endl << endl;
	return Rect(x, y, width, height);
}
void OfflineMode::Run()
{	
	VideoReader* reader;
	reader = new VideoReader(_videoFileName);
	cout << reader->StartRead() << endl;
	int dataQuantity = reader->GetDataQuantity();	
	VideoWriter writer;
	string name = "0116_12";
	string resultFileName= name+"_result.avi";
	writer.open(resultFileName, CV_FOURCC('M', 'J', 'P', 'G'), reader->GetCameraFPS()*2, reader->getVideoSize());
	if (!writer.isOpened())
	{
		return;
	}
	
	/*int filecount = 0;
	int objectSum = 0;
	fstream fp;
	fp.open(name+".txt", ios::out);
	if (!fp) {
		cout << "Fail to open file: "<< endl;
	}*/
	

	for (int i = 0; i < dataQuantity; i++)
	{			
		Mat frame;				
		Mat grayFrame;		
		if (!reader->RequestData(frame)) 
		{
			continue;
		}

		//_waitKeySec = 0;
		//Mat dst;
		//std::vector<cv::Size> size;
		//size.push_back(Size(640,480));
		//size.push_back(Size(533, 400));
		//size.push_back(Size(444, 333));
		//size.push_back(Size(370, 278));
		//size.push_back(Size(309,231));
		//size.push_back(Size(257,193));
		//size.push_back(Size(214,161));
		//size.push_back(Size(179,134));
		//size.push_back(Size(149,112));
		//int area = 0;
		//for (int j = 0;j<size.size(); j++)
		//{
		//	resize(frame, dst,size[j]);
		//				
		//	stringstream s;
		//	s << j;
		//	//imshow(_videoFileName + s.str(), dst(test(dst)));
		//	//cv::rectangle(dst,Rect(0, dst.rows*0.3, dst.cols, dst.rows*0.7), Scalar(255, 0, 0), 2);
		//	//cout << "aaarea" << Rect(0, dst.rows*0.3, dst.cols, dst.rows*0.7).area() << endl;
		//	//cv::rectangle(dst, test(dst), Scalar(255, 0, 0), 2);			
		//	//area+= Rect(0, dst.rows*0.3, dst.cols, dst.rows*0.7).area();
		//	//Size temp = size[j];
		//	
		//	Mat f = frame(test(dst));
		//	//Mat f = frame(Rect(0, dst.rows*0.3, dst.cols, dst.rows*0.7) );
		//	cvtColor(f, grayFrame, CV_BGR2GRAY);
		//	frontbackobject = ((SvmClassifier*)_classifierList[0])->ClassifyTest(f, grayFrame);
		//	//cout <<"面積"<< test(dst).area() << endl;
		//	//line(dst, cv::Point(0, dst.rows*0.5), cv::Point(dst.cols, dst.rows*0.5), Scalar(0, 255, 0),2);//地平線
		//	//Detect(frame, grayFrame, i);
		//	imshow(_videoFileName + s.str(), dst);
		//}
		

		//Mat resizeframe;
		//resize(frame, resizeframe, Size(), 0.4, 0.4);
		//cvtColor(resizeframe, grayFrame, CV_BGR2GRAY);
		//double t = (double)cv::getTickCount();
		////unsigned long start = clock();
		//Detect(resizeframe, grayFrame, i);
		////unsigned long end = clock();
		//t = (double)cv::getTickCount() - t;
		//for (int i = 0;i<object.size(); i++)
		//{
		//	Rect temp = Rect(object[i].x*2.5, object[i].y*2.5, object[i].width*2.5, object[i].height*2.5);
		//	cv::rectangle(frame, temp, Scalar(255,255,255), 2);
		//}

				
		cvtColor(frame, grayFrame, CV_BGR2GRAY);
		double t = (double)cv::getTickCount();		
		//unsigned long start = clock();
		Detect(frame, grayFrame, i);
		//unsigned long end = clock();
		t = (double)cv::getTickCount() - t;
	
		for (int j = 0; j<frontbackobject.size(); j++)
		{			
			cv::rectangle(frame, frontbackobject[j], Scalar(0,255,0), 2);			
			//objectSum++;
		}	
		for (int j = 0; j<sideobject.size(); j++)
		{
			cv::rectangle(frame, sideobject[j], Scalar(255, 0, 0), 2);
		}
		for (int j = 0; j<pedobject.size(); j++)
		{
			cv::rectangle(frame, pedobject[j], Scalar(0, 255, 255), 2);
		}
		frontbackobject.clear();
		sideobject.clear();
		pedobject.clear();
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
		//putText(frame, ss2.str(), CvPoint(0, frame.rows - 50), 0, 1, Scalar(255, 255, 255), 1, 8, false);		
		
		//cout << "frame:" << i << endl << "FPS:" << dt << endl;
		#endif // fpsImformation

		
		std::string name = "temp\\" + ss2.str() + ".jpg";
		cv::imwrite(name, frame);
		
		//_waitKeySec = 0;
		imshow(_videoFileName, frame);
		writer << frame;
	

		waitKey(_waitKeySec);
	}
	//fp <<"tp:"<<objectSum<< endl;//寫入字串			
	//fp.close();
	//cout << "抓到" << objectSum <<endl;
	destroyAllWindows();
}

void OfflineMode::RunCompare() 
{
	VideoReader* reader;
	reader = new VideoReader(_videoFileName);
	cout << reader->StartRead() << endl;
	int dataQuantity = reader->GetDataQuantity();


	VideoReader* reader2;
	reader2 = new VideoReader(_videoFileName);
	cout << reader2->StartRead() << endl;



	VideoWriter writer;
	string name = "1220_10";
	string resultFileName = name + "_resultCompare.avi";
	Size videoSize = Size(reader->getVideoSize().width * 2, reader->getVideoSize().height);
	writer.open(resultFileName, CV_FOURCC('M', 'J', 'P', 'G'), reader->GetCameraFPS() * 2, videoSize);
	if (!writer.isOpened())
	{
		return;
	}

	int filecount = 0;
	int objectSum = 0;
	int objectSum2 = 0;
	fstream fp;
	fp.open(name + ".txt", ios::out);
	if (!fp) {
		cout << "Fail to open file: " << endl;
	}

	for (int i = 0; i < dataQuantity; i++)
	{
		Mat frame;
		Mat grayFrame;
		if (!reader->RequestData(frame))
		{
			continue;
		}

		cvtColor(frame, grayFrame, CV_BGR2GRAY);
		double t = (double)cv::getTickCount();
		frontbackobject = ((SvmClassifier*)_classifierList[0])->ClassifyTest(frame, grayFrame);//new
		t = (double)cv::getTickCount() - t;

		for (int j = 0; j<frontbackobject.size(); j++)
		{
			cv::rectangle(frame, frontbackobject[j], Scalar(0, 255, 0), 2);
			std::stringstream s;
			s << objectSum;
			std::string samplename = "result\\" +name+"\\mine\\" + s.str() + ".jpg";						
			cv::imwrite(samplename, frame(frontbackobject[j]));
			objectSum++;
		}
		for (int j = 0; j<sideobject.size(); j++)
		{
			cv::rectangle(frame, sideobject[j], Scalar(255, 0, 0), 2);
		}
		for (int j = 0; j<pedobject.size(); j++)
		{
			cv::rectangle(frame, pedobject[j], Scalar(0, 255, 255), 2);
		}

		int dt = int(cv::getTickFrequency() / t);
		std::stringstream ss;
		ss << dt;
		std::string fpsString("FPS:");
		fpsString += ss.str();		
		putText(frame, fpsString, CvPoint(0, frame.rows - 25), 0, 1, Scalar(255, 255, 255), 1, 8, false);



		Mat frame2;
		Mat grayFrame2;
		if (!reader2->RequestData(frame2))
		{
			continue;
		}

		cvtColor(frame2, grayFrame2, CV_BGR2GRAY);
		double t2 = (double)cv::getTickCount();
		frontbackobject = ((SvmClassifier*)_classifierList[3])->ClassifyTest(frame2, grayFrame2);//old
		t2 = (double)cv::getTickCount() - t2;

		for (int j = 0; j<frontbackobject.size(); j++)
		{
			cv::rectangle(frame2, frontbackobject[j], Scalar(0, 255, 0), 2);
			std::stringstream s;
			s << objectSum;
			std::string samplename = "result\\" + name + "\\opencv\\" + s.str() + ".jpg";
			cv::imwrite(samplename, frame2(frontbackobject[j]));									
			objectSum2++;
		}
		for (int j = 0; j<sideobject.size(); j++)
		{
			cv::rectangle(frame2, sideobject[j], Scalar(255, 0, 0), 2);
		}
		for (int j = 0; j<pedobject.size(); j++)
		{
			cv::rectangle(frame2, pedobject[j], Scalar(0, 255, 255), 2);
		}

		int dt2 = int(cv::getTickFrequency() / t2);
		std::stringstream ss2;
		ss2 << dt2;
		std::string fpsString2("FPS:");
		fpsString2 += ss2.str();		
		putText(frame2, fpsString2, CvPoint(0, frame2.rows - 25), 0, 1, Scalar(255, 255, 255), 1, 8, false);

		//_waitKeySec = 0;		
		
		Mat newimg = Mat(frame.rows, frame.cols * 2, frame.type());

		Mat tmpimg;
		
		tmpimg = newimg(Rect(0, 0, frame.cols, frame.rows));
		frame.copyTo(tmpimg);

		tmpimg = newimg(Rect(frame.cols, 0, frame.cols, frame.rows));
		frame2.copyTo(tmpimg);


		imshow("LEFT NEW RIGHT OLD", newimg);
		writer << newimg;

		waitKey(_waitKeySec);
	}
	//fp << "tp:" << objectSum << endl;//寫入字串			
	//fp.close();
	//cout << "抓到" << objectSum << endl;
	destroyAllWindows();
}

























