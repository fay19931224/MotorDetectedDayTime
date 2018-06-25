#include "SvmClassifier.h"

#define draw
#define drawHelmet
//#define drawImformation
//#define drawPredictDirect
//#define drawFilterOut
//#define drawBye

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
	_descriptor.setHroizon( _hogParameter.horion);
	
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
	_descriptor.setHroizon( _hogParameter.horion);	

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

void SvmClassifier::Classify(Mat & frame, Mat & grayFrame)
{		
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(frame);
		TrackingObject* tempMotorcyclist = temp[i]->GetObject(ObjectType);	
		if (tempMotorcyclist->confidence() > 0.2)
		{
			/*Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
			HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();
			if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct))
			{
				circle(frame, _headDetectReturnStruct->center, _headDetectReturnStruct->radius, Scalar(255, 255, 255), 2, 8, 0);
			}*/
#ifdef draw
			temp[i]->DrawObj(frame);
#endif
			_trackingObject.push_back(temp[i]);
		}
	}

	for (int i = 0; i < _result.size(); i++)
	{
		for (int j = 0; j < _trackingObject.size(); j++)
		{
			TrackingObject *temp = _trackingObject[j]->GetObject(ObjectType);			
			Rect intersection = _result[i] & temp->getROI();
			if (intersection.area() != 0)
			{
				cv::Point tempCenter((temp->getROI().br()+ temp->getROI().tl())/2);
				cv::Point resultCenter((_result[i].br() + _result[i].tl()) / 2);
				if (intersection.contains(tempCenter)&& intersection.contains(resultCenter))
				{		
					temp->isDetected = true;
					temp->tracker->init(_result[i], frame);										
					_result[i] = Rect(0, 0, 0, 0);
					break;
				}
			}
		}		
		if (_result[i].area() == 0 ||(_result[i].y + _result[i].height) <= frame.rows * 4 / 10 || _result[i].height >= frame.rows / 2)
		{
			continue;
		}
		Rect tempROI = checkROI(_result[i], grayFrame);

		HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();
		if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct))
		{
			Motorcyclist* motorcyclist = new Motorcyclist(frame, _result[i], _headDetectReturnStruct, _rectangleColor, Scalar(0, 0, 255));
#ifdef draw
			motorcyclist->DrawObj(frame);
#ifdef drawHelmet
			motorcyclist->DrawObjHead(frame);
#endif
#endif				
			/*stringstream ss;
			ss << index++;
			stringstream ss2;
			ss2 << _type;
			string name = ss.str() + ss2.str() + ".jpg";
			imwrite(name, frame); */
			_trackingObject.push_back(motorcyclist);
		}
	}	
	
	/*for (int i = 0; i < _trackingObject.size(); i++)
	{
		TrackingObject *temp = _trackingObject[i]->GetObject(ObjectType);
		if (!temp->isDetected)
		{
			temp->missCount++;
		}
		temp->isDetected = false;
	}*/
}

void SvmClassifier::ClassifyPedes(Mat & frame, Mat & grayFrame)
{		
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(frame);
		TrackingObject* tempCar = temp[i]->GetObject(ObjectType);				
		if ((tempCar->confidence() > 0.2))
		{
#ifdef draw
			temp[i]->DrawObj(frame);					
#endif
			_trackingObject.push_back(temp[i]);
		}
	}	
	
	for (int i = 0; i < _result.size(); i++)
	{
		for (int j = 0; j < _trackingObject.size(); j++)
		{
			TrackingObject *temp = _trackingObject[j]->GetObject(ObjectType);
			Rect intersection = _result[i] & temp->getROI();
			if (intersection.area() != 0)
			{
				cv::Point tempCenter((temp->getROI().br() + temp->getROI().tl()) / 2);
				cv::Point resultCenter((_result[i].br() + _result[i].tl()) / 2);
				if (intersection.contains(tempCenter) && intersection.contains(resultCenter))
				{				
					temp->isDetected = true;
					temp->tracker->init(_result[i], frame);
					_result[i] = Rect(0, 0, 0, 0);
					break;
				}
			}
		}
		if (_result[i].area() == 0)
		{
			continue;
		}
		DetectedPeople* detectedPed = new DetectedPeople(frame, _result[i], _rectangleColor);
#ifdef draw
		detectedPed->DrawObj(frame);
#endif	
		_trackingObject.push_back(detectedPed);
	}
}

void SvmClassifier::Update_track(Mat &frame)
{				
	/*Mat grayFrame;
	cvtColor(frame, grayFrame, CV_BGR2GRAY);*/
	for (int i = 0; i<_trackingObject.size(); i++)
	{								
		_trackingObject[i]->UpdateObj(frame);
		TrackingObject* tempMotorcyclist = _trackingObject[i]->GetObject(ObjectType);		

		if (!isOutOfRange(tempMotorcyclist->getROI(), frame))
		{			
			//Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
			//HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();
			//if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)) 
			//{
			//	((Motorcyclist*)_trackingObject[i])->setHeadStruct(_headDetectReturnStruct->center, _headDetectReturnStruct->radius);
			//	#ifdef draw									
			//	_trackingObject[i]->DrawObj(frame);
			//	#ifdef drawHelmet
			//	((Motorcyclist*)_trackingObject[i])->DrawObjHead(frame);
			//	#endif // drawhelmet
			//	#endif // drawhelmet				
			//}			
			#ifdef draw						
			_trackingObject[i]->DrawObj(frame);				
			#endif
		}		
	}	
}

void SvmClassifier::Update_trackPedes(Mat & frame)
{		
	for (int i = 0; i < _trackingObject.size(); i++)
	{
		if (_trackingObject[i]->isduplicate) 
		{
			continue;
		}
		for (int j = i+1; j < _trackingObject.size(); j++)
		{
			TrackingObject *tempI = _trackingObject[i]->GetObject(ObjectType);
			TrackingObject *tempJ = _trackingObject[j]->GetObject(ObjectType);
			Rect intersection = tempI->getROI()& tempJ->getROI();
			if (intersection.area() != 0)
			{
				cv::Point tempICenter((tempI->getROI().br() + tempI->getROI().tl()) / 2);
				cv::Point tempJCenter((tempJ->getROI().br() + tempJ->getROI().tl()) / 2);
				if (intersection.contains(tempICenter) && intersection.contains(tempJCenter))
				{									
					_trackingObject[i]->isduplicate = true;										
				}
			}
		}

	}

	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	for (int i = 0; i<temp.size(); i++)
	{
		temp[i]->UpdateObj(frame);
		if (!isOutOfRange(_trackingObject[i]->GetObject(ObjectType)->getROI(), frame)&& !_trackingObject[i]->isduplicate)
		{			
#ifdef draw						
			_trackingObject[i]->DrawObj(frame);				
			_trackingObject[i]->isduplicate = false;
			_trackingObject.push_back(temp[i]);
#endif		
		}
	}	
}

void SvmClassifier::ClassifyTest(Mat & frame, Mat & grayFrame)
{	
	static int  count = 0;
	unsigned long start = clock();	
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);
	unsigned long end = clock();
	//printf("total time=%1.3f seconds\n", (end - start) / 1000.0);
	for (int i = 0;i< _result.size(); i++)
	{			
		std::stringstream ss;
		/*ss << count++;
		cv::imwrite("pic\\9\\"+ss.str()+".jpg", frame(_result[i]));*/
		cv::rectangle(frame, _result[i], _rectangleColor, 2);		
	}
}


Rect SvmClassifier::checkROI(Rect roi,Mat frame)
{
	
	int x = roi.x+ roi.width/4;
	int y = roi.y;
	int width = roi.width/2;
	int height = roi.height;
	switch (_type)
	{
	case MotorbikeFrontBack:	
		if (y - 10 > 0)
		{
			y -= 10;
		}
		else
		{
			y = 0;;
		}
		break;
	case MotorbikeSide:
		x = roi.x + roi.width / 5;;
		width = roi.width- roi.width*2 / 5;
		height = (roi.height*3)/5;
		break;	
	default:
		break;
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