#include "SvmClassifier.h"

#define draw
#define drawHelmet
//#define drawImformation
//#define drawPredictDirect
//#define drawFilterOut
//#define drawBye

//for thread
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter svmDetectParameter, HeadDetecter* headdetectd, VideoReader* videoReader) : Classifier(type, rectangleColor)
{	
	_type = type;
	setObjectType();	
	_headDetected = headdetectd;
	_hogParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.WINDOW_SIZE;
	Size _blockSize = Size(_hogParameter.CELL_SIZE.width * 2, _hogParameter.CELL_SIZE.height * 2);
	Size _blockStride = _hogParameter.CELL_SIZE;
	Size _cellSize = _hogParameter.CELL_SIZE;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_videoReader = videoReader;
	_descriptor = HOGDescriptor(_hogParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	_descriptor.setSVMDetector(hogVector);
	_descriptor.setVersion(true);	
	_ulmutex = std::unique_lock<std::mutex>(_mutex);
}

SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter svmDetectParameter, VideoReader* videoReader) : Classifier(type, rectangleColor)
{
	_type = type;
	setObjectType();	
	_hogParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.WINDOW_SIZE;
	Size _blockSize = Size(_hogParameter.CELL_SIZE.width * 2, _hogParameter.CELL_SIZE.height * 2);
	Size _blockStride = _hogParameter.CELL_SIZE;
	Size _cellSize = _hogParameter.CELL_SIZE;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_videoReader = videoReader;
	_descriptor = HOGDescriptor(_hogParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	_descriptor.setSVMDetector(hogVector);
	_descriptor.setVersion(true);	
	_ulmutex = std::unique_lock<std::mutex>(_mutex);
}



/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter svmDetectParameter, FusionManager* fusionManager, HeadDetecter* headdetectd) : Classifier(type, rectangleColor)
{				
	_type = type;	
	setObjectType();
	_fusionManager = fusionManager;
	_headDetected=headdetectd;
	_hogParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.WINDOW_SIZE;
	Size _blockSize = Size(_hogParameter.CELL_SIZE.width * 2, _hogParameter.CELL_SIZE.height * 2);
	Size _blockStride = _hogParameter.CELL_SIZE;
	Size _cellSize = _hogParameter.CELL_SIZE;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_hogParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);		
	_descriptor.setSVMDetector(hogVector);
	_descriptor.setVersion(true);
	
	t1 = nullptr;	
	t2 = nullptr;		
}

SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter svmDetectParameter, FusionManager* fusionManager) : Classifier(type, rectangleColor)
{	
	_type = type;
	setObjectType();
	_fusionManager = fusionManager;
	_hogParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.WINDOW_SIZE;
	Size _blockSize = Size(_hogParameter.CELL_SIZE.width * 2, _hogParameter.CELL_SIZE.height * 2);
	Size _blockStride = _hogParameter.CELL_SIZE;
	Size _cellSize = _hogParameter.CELL_SIZE;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_hogParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	_descriptor.setSVMDetector(hogVector);
	_descriptor.setVersion(false);
	
	t1 = nullptr;
	t2 = nullptr;
	
}



SvmClassifier::~SvmClassifier()
{
	stop();
}

bool SvmClassifier::startClassify(Mat &frame,Mat & grayFrame)
{	
	if (!t1)
	{
		void (SvmClassifier::*myFunc)(Mat &frame, Mat &grayFrame);
		switch (_type)
		{
		case MotorbikeFrontBack:			
		case MotorbikeSide:
			myFunc= &SvmClassifier::Classify;			
			//myFunc = &SvmClassifier::ClassifyTest;
			break;		
		case Pedestrian:
			myFunc = &SvmClassifier::ClassifyPedes;
			break;
		default:
			throw exception("classify error");						
		}
		
		t1 = new std::thread(myFunc, this, std::ref(frame), std::ref(grayFrame));		
	}
	return t1 != nullptr;
}

bool SvmClassifier::startUpdateTrack(Mat & frame)
{
	if (!t2)
	{
		void (SvmClassifier::*myFunc)(Mat &frame);
		switch (_type)
		{
		case MotorbikeFrontBack:
		case MotorbikeSide:
			myFunc = &SvmClassifier::Update_track;
			break;		
		case Pedestrian:
			myFunc = &SvmClassifier::Update_trackPedes;
			break;
		default:
			throw exception("classify error");
		}
		t2 = new std::thread(myFunc, this, std::ref(frame));
	}
	return t2 != nullptr;
}

bool SvmClassifier::stop()
{
	if(t1)
	{		
		t1->join();
		
		delete t1;
		t1 = nullptr;
	}
	if (t2)
	{
		t2->join();
		delete t2;
		t2 = nullptr;
	}
	return true;
}

void SvmClassifier::Classify(Mat &frame,Mat &grayFrame)
{				
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();	
	Mat tempFrame = frame.clone();
	for (int i = 0; i < temp.size(); i++)
	{				
		temp[i]->UpdateObj(tempFrame);
		TrackingObject* tempMotorcyclist = temp[i]->GetObject(ObjectType);
		
		if ((tempMotorcyclist->confidence() > 0.3)&&(tempMotorcyclist->missCount<3)
			&&(tempMotorcyclist->getROI().y + tempMotorcyclist->getROI().height) > frame.rows *4/10 && tempMotorcyclist->getROI().height < frame.rows / 2)
		{						
			HeadDetectReturnStruct* _headDetectReturnStruct=new HeadDetectReturnStruct();
			Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);				
			if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)) 
			{
				((Motorcyclist*)temp[i])->setHeadStruct(_headDetectReturnStruct->center, _headDetectReturnStruct->radius);
			}			
			#ifdef draw
			temp[i]->DrawObj(frame);					
			#ifdef drawHelmet
				((Motorcyclist*)temp[i])->DrawObjHead(frame);
			#endif		
			#ifdef drawImformation
			std::stringstream ss2;
			ss2 << tempMotorcyclist->confidence();
			putText(frame, "t_hc", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			putText(frame, ss2.str().substr(0, 4), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			#endif			
			#endif
			
			_trackingObject.push_back(temp[i]);			
		}
	}
	
	_descriptor.detectMultiScale(grayFrame,_result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	refineROI(_result, _trackingObject);				
	
	for (int i = 0; i<_result.size(); i++)
	{		
		if ((_result[i].y + _result[i].height) <= frame.rows*4/10 || _result[i].height >= frame.rows/2)
		{
			continue;
		}
		Rect tempROI= checkROI(_result[i], grayFrame);								
		#ifdef draw
		#ifdef drawFilterOut		
		cv::rectangle(frame, _result[i], Scalar(0, 0, 0), 2);
		#endif
		#endif
		//saveImage(tempFrame(tempROI),"half");
		
		HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();

		Motorcyclist* motorcyclist = new Motorcyclist(tempFrame, _result[i], _headDetectReturnStruct, _rectangleColor, Scalar(0, 0, 255));
		motorcyclist->DrawObj(frame);
		_trackingObject.push_back(motorcyclist);

		if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct))
		{				
			Motorcyclist* motorcyclist = new Motorcyclist(tempFrame, _result[i], _headDetectReturnStruct, _rectangleColor, Scalar(0, 0, 255));
			#ifdef draw
				motorcyclist->DrawObj(frame);						
			#ifdef drawHelmet
				motorcyclist->DrawObjHead(frame);
			#endif // drawHelmet			
			#ifdef drawImformation						
			putText(frame, "SVM", _result[i].tl(), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			/*std::stringstream ss;
			ss << foundweight[i];
			string temp ="w:"+ ss.str();
			putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 25), 1, 8, false);*/
			#endif						
			#endif						
			_trackingObject.push_back(motorcyclist);
		}
	}	
	
}

void SvmClassifier::ClassifyPedes(Mat & frame, Mat & grayFrame)
{
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	Mat tempFrame = frame.clone();
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(tempFrame);
		TrackingObject* tempCar = temp[i]->GetObject(ObjectType);
		
		if ((tempCar->confidence() > 0.3) &&(tempCar->missCount<4))
		{
#ifdef draw
			temp[i]->DrawObj(frame);					
#ifdef drawImformation
			std::stringstream ss2;
			ss2 << tempCar->confidence();
			putText(frame, "t_hc", tempCar->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			putText(frame, ss2.str().substr(0, 4), CvPoint(tempCar->getROI().x, tempCar->getROI().y + 20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
#endif
#endif
			_trackingObject.push_back(temp[i]);
		}
	}

	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);	
	refineROI(_result, _trackingObject);

	for (int i = 0; i<_result.size(); i++)
	{		
		DetectedCarAndPeople* detectedCar = new DetectedCarAndPeople(tempFrame, _result[i], _rectangleColor);
#ifdef draw
		detectedCar->DrawObj(frame);			
		_trackingObject.push_back(detectedCar);
#ifdef drawImformation						
		putText(frame, "SVM", _result[i].tl(), 0, 1, Scalar(255, 122, 255), 1, 8, false);
		/*std::stringstream ss;
		ss << foundweight[i];
		string temp ="w:"+ ss.str();
		putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 25), 1, 8, false);*/
#endif			
#endif		
	}		
}

void SvmClassifier::Update_track(Mat &frame)
{				
	Mat tempFrame = frame.clone();
	Mat grayFrame;
	cvtColor(frame, grayFrame, CV_BGR2GRAY);
	for (int i = 0; i<_trackingObject.size(); i++)
	{								
		_trackingObject[i]->UpdateObj(tempFrame);		
		TrackingObject* tempMotorcyclist = _trackingObject[i]->GetObject(ObjectType);
		if (_type == ClassiferType::MotorbikeSide&&abs(tempMotorcyclist->initRect.x - tempMotorcyclist->getROI().x)<10)
		{
			continue;
		}

		if (!isOutOfRange(tempMotorcyclist->getROI(), frame))
		{			
			if ((tempMotorcyclist->getROI().y + tempMotorcyclist->getROI().height) <= frame.rows * 4 / 10 || tempMotorcyclist->getROI().height >= frame.rows / 2)
			{
				continue;
			}			
			//Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
			//if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)) 
			//{
			//	((Motorcyclist*)_trackingObject[i])->setHeadStruct(_headDetectReturnStruct->center, _headDetectReturnStruct->radius);
			//	#ifdef draw										
			//	#ifdef drawHelmet
			//	((Motorcyclist*)_trackingObject[i])->DrawObjHead(frame);
			//	#endif // drawhelmet
			//	#endif // drawhelmet				
			//}
			//
			#ifdef draw						
			_trackingObject[i]->DrawObj(frame);						
			
			#ifdef drawImformation						
			putText(frame, "t", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
			#endif				
			#endif					
		}		
	}
}

void SvmClassifier::Update_trackPedes(Mat & frame)
{	
	Mat tempFrame = frame.clone();	
	for (int i = 0; i<_trackingObject.size(); i++)
	{
		_trackingObject[i]->UpdateObj(tempFrame);

		if (!isOutOfRange(_trackingObject[i]->GetObject(ObjectType)->getROI(), frame))
		{			
#ifdef draw						
			_trackingObject[i]->DrawObj(frame);												
#ifdef drawImformation						
			putText(frame, "t", _trackingObject[i]->GetObject(ObjectType)->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
#endif	
#endif		
		}
	}
}

void SvmClassifier::setThread(thread * thread)
{
	t1 = thread;
}

void SvmClassifier::setFrame(Mat & frame)
{
	_frame = frame;
}

void SvmClassifier::drawObject(Mat& frame)
{
	for (int i = 0; i<_objectPosition.size(); i++)
	{
		cv::rectangle(frame, _objectPosition[i], _rectangleColor, 2);
	}
}

void SvmClassifier::ClassifyTest(Mat & frame, Mat & grayFrame)
{	
	static int  count = 0;
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	for (int i = 0;i< _result.size(); i++)
	{			
		std::stringstream ss;
		/*ss << count++;
		cv::imwrite("pic\\9\\"+ss.str()+".jpg", frame(_result[i]));*/
		cv::rectangle(frame, _result[i], _rectangleColor, 2);		
	}
}

void SvmClassifier::useThread()
{		
	for (int i = 0; _videoReader->GetDataQuantity(); i++)
	{
		_start = false;
		
		_cond->wait(_ulmutex, [this] {printf("%d wait\n",_type); _mainStartFlag = true; return *_mainFlag; });				
		_start = true;
		_childDone = false;
		
		printf("%d_doing %d\n", _type, i);
		//Classify(_videoReader->getFrame(), _videoReader->getGrayFrame());
		//std::this_thread::sleep_for(1s);

		_childDone = true;
		while (_mainStartFlag)
		{			
			printf("%d_waitsetting %d\n", _type,i);
		}
		printf("%d_done %d\n", _type, i);
	}	
}

vector<Rect> SvmClassifier::getObjectPosition()
{
	return _objectPosition;
}

Rect SvmClassifier::checkROI(Rect roi,Mat frame)
{
	
	int x = roi.x;
	int y = roi.y;
	int width = roi.width;
	int height = roi.height;
	if (y - 20 > 0)
	{
		y -= 20;
	}
	else
	{
		y = 0;;
	}
	return Rect(x, y, width, height / 2);
	
}

bool SvmClassifier::isOutOfRange(Rect roi,Mat frame)
{		
	return (roi.x<0)|| (roi.y<0)|| (roi.x+roi.width>frame.cols)|| (roi.y + roi.height>frame.rows) ? true:false;
}

void SvmClassifier::showLidarInformation(Mat &frame,Rect &roi,int distant)
{
	if (!_fusionManager) {
		return;
	}	
	std::stringstream ss;
	ss << distant;
	string temp = ss.str()+"m";
	putText(frame, temp, CvPoint(roi.x,roi.y+roi.height-20), 0, 1, Scalar(0, 122, 255), 1, 8, false);
}

int SvmClassifier::getLiadarDistant(Mat frame, Rect roi)
{
	if (!_fusionManager) 
	{
		return -2;
	}	
	Rect temp = roi;	
	if (roi.x + roi.width / 2 < frame.cols / 3)
	{		
		if (temp.x + temp.width * 3 / 2 < frame.cols)
		{
			roi.width += roi.width / 2;
		}
	}
	else  if(roi.x + roi.width / 2 > frame.cols*2 / 3)
	{	
		if (temp.x - temp.width / 2 >= 0)
		{
			roi.x -= roi.width / 2;
			roi.width += roi.width / 2;
		}		
	}	
	int distant =(int)( _fusionManager->RequestDistance(frame, roi)) / 1000;
	if (distant > 30) 
	{
		distant = -1;
	}
	return distant;
}

/*
result要濾除重複的框
result裡跟trackingObject一樣的框砍掉
*/
void SvmClassifier::refineROI(vector<Rect> &result, vector<DetectedObject*>  &trackingObject)
{		
	for (int i = 0; i < result.size(); i++)
	{		
		for (int j = i+1; j < result.size(); j++)
		{
			Rect intersection = result[i] & result[j];			
			if (intersection.area() != 0)
			{								
				float resultIArea = static_cast<float>(result[i].area());
				float resultJArea = static_cast<float>(result[j].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if ((intersectionArea / resultIArea)>0.3|| (intersectionArea / resultJArea)>0.3)
				{					
					result[i] = result[i] | result[j];
					result[j] = Rect(0, 0, 0, 0);						
				}
			}
		}
	}
	
	for (int i = 0; i < result.size(); i++)
	{
		if (result[i].area() != 0)
		{
			for (int j = 0; j < trackingObject.size(); j++)
			{			
				TrackingObject *temp = trackingObject[j]->GetObject(ObjectType);
				Rect intersection = result[i] & temp->getROI();
				if (intersection.area() != 0) 
				{
					float resultArea = static_cast<float>(result[i].area());
					float trackingObjectArea = static_cast<float>(temp->getROI().area());
					float intersectionArea = static_cast<float>(intersection.area());
					if ((intersectionArea / resultArea)>0.5 || (intersectionArea / trackingObjectArea)>0.5)
					{
						temp->isDetected = true;
						result[i] = Rect(0, 0, 0, 0);
					}
				}				
			}
		}
	}


	for (int i = 0; i < trackingObject.size(); i++)
	{
		TrackingObject *temp = trackingObject[i]->GetObject(ObjectType);
		if (!temp->isDetected)
		{
			temp->missCount++;			
			temp->isDetected = false;
		}
	}

	for (vector<Rect>::iterator iter = result.begin(); iter != result.end();)
	{		
		if ((Rect(*iter)).area() == 0)
			iter = result.erase(iter);
		else
		{
			iter++;
		}
	}
}

void SvmClassifier::setObjectType()
{
	switch (_type)
	{
	case MotorbikeFrontBack:		
	case MotorbikeSide:
		ObjectType = "moto";
		break;
	case CarFrontBack:
		ObjectType = "car";
		break;
	case Pedestrian:
		ObjectType = "Pedestrian";
		break;
	case Helmet:
		ObjectType = "head";
		break;
	default:
		break;
	}
}
