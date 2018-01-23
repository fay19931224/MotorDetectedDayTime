﻿#include "SvmClassifier.h"

#define draw
//#define drawHelmet
//#define drawImformation
#define drawPredictDirect
//#define drawFilterOut
//#define UdpSocketServer
//#define drawBye
/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor,svmDetectParameter svmDetectParameter, FusionManager* fusionManager, HeadDetecter* headdetectd) : Classifier(type, rectangleColor)
{				
	_type = type;	
	setObjectType();
	_fusionManager = fusionManager;
	_headDetected=headdetectd;
	_svmDetectParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _svmDetectParameter.WINDOW_SIZE;
	Size _blockSize = Size(_svmDetectParameter.CELL_SIZE.width * 2, _svmDetectParameter.CELL_SIZE.height * 2);
	Size _blockStride = _svmDetectParameter.CELL_SIZE;
	Size _cellSize = _svmDetectParameter.CELL_SIZE;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_svmDetectParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, 9, 1, -1, HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);	
	_descriptor.setSVMDetector(hogVector);
	t1 = nullptr;	
	t2 = nullptr;	
	_SentData = new vector<SentData>();
}

SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter, FusionManager* fusionManager) : Classifier(type, rectangleColor)
{
	_type = type;
	setObjectType();
	_fusionManager = fusionManager;
	_svmDetectParameter = svmDetectParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _svmDetectParameter.WINDOW_SIZE;
	Size _blockSize = Size(_svmDetectParameter.CELL_SIZE.width * 2, _svmDetectParameter.CELL_SIZE.height * 2);
	Size _blockStride = _svmDetectParameter.CELL_SIZE;
	Size _cellSize = _svmDetectParameter.CELL_SIZE;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_svmDetectParameter.WINDOW_SIZE, _blockSize, _blockStride, _cellSize, 9, 1, -1, HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	_descriptor.setSVMDetector(hogVector);
	t1 = nullptr;
	t2 = nullptr;
	_SentData = new vector<SentData>();
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
		case CarFrontBack:
			myFunc= &SvmClassifier::ClassifyCar;
			break;
		case Helmet:
			break;
		case Pedestrian:
			myFunc = &SvmClassifier::ClassifyCar;
			break;
		default:
			cout << "error" << endl;
			break;
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
		case CarFrontBack:
			myFunc = &SvmClassifier::Update_trackCar;
			break;
		case Helmet:
			break;
		case Pedestrian:
			myFunc = &SvmClassifier::Update_trackCar;
			break;
		default:
			cout << "error" << endl;
			break;
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
	_SentData->clear();
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();	
	Mat tempFrame = frame.clone();
	for (int i = 0; i < temp.size(); i++)
	{				
		temp[i]->UpdateObj(tempFrame);
		TrackingObject* tempMotorcyclist = temp[i]->GetObject(ObjectType);
		
		int distant=-1;
		if (!isOutOfRange(tempMotorcyclist->getROI(), frame)) 
		{
			distant = getLiadarDistant(frame, tempMotorcyclist->getROI());
		}		

		if ((tempMotorcyclist->confidence() > 0.35)&&(tempMotorcyclist->missCount<3)&& (distant !=-1)&&(tempMotorcyclist->getROI().area()>2000)
			&&(tempMotorcyclist->getROI().y + tempMotorcyclist->getROI().height) > frame.rows *4/10 && tempMotorcyclist->getROI().height < frame.rows / 2)
		{			
			if (_type == ClassiferType::MotorbikeSide&&abs(tempMotorcyclist->initRect.x - tempMotorcyclist->getROI().x)<10)
			{				
				continue;
			}

			HeadDetectReturnStruct* _headDetectReturnStruct=new HeadDetectReturnStruct();
			Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);				
			if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)) 
			{
				((Motorcyclist*)temp[i])->setHeadStruct(_headDetectReturnStruct->center, _headDetectReturnStruct->radius);
			}			
			#ifdef draw
			temp[i]->DrawObj(frame);	
			showLidarInformation(frame, tempMotorcyclist->getROI(), distant);
			#ifdef drawHelmet
				((Motorcyclist*)temp[i])->DrawObjHead(frame);
			#endif		
			#ifdef drawImformation
			std::stringstream ss2;
			ss2 << tempMotorcyclist->confidence();
			putText(frame, "t_hc", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			putText(frame, ss2.str().substr(0, 4), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			#endif
			#ifdef drawPredictDirect
			if (_type == ClassiferType::MotorbikeFrontBack)
			{
				putText(frame, temp[i]->predictDirect(), CvPoint(tempMotorcyclist->getROI().x + 20, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			}
			#endif 
			#endif
						
			#ifdef UdpSocketServer						
			_SentData->push_back(setSentData(tempMotorcyclist->getROI(), distant));
			#endif // UdpSocketServer	

			
			_trackingObject.push_back(temp[i]);
			
		}
		#ifdef drawBye
		else
		{						
			bool b=isOutOfRange(tempMotorcyclist->getROI(), frame);			

			temp[i]->DrawObj(frame);
			putText(frame, "bye", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(122, 100, 100), 1, 8, false);			
			std::stringstream ss2;
			ss2 << tempMotorcyclist->confidence();
			putText(frame, ss2.str().substr(0,3), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			
			std::stringstream ss3;
			ss3 << b;			
			putText(frame, ss3.str(), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 60), 0, 1, Scalar(122, 100, 100), 1, 8, false);			
		}
		#endif
	}
	
	_descriptor.detectMultiScale(grayFrame,_result, foundweight, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);

	for (int i = 0; i < _result.size(); i++) 
	{		
		saveImage(frame(_result[i]), "=");
	}
	//cv::line(frame, cv::Point(0, frame.rows / 2), cv::Point(frame.cols, frame.rows / 2), Scalar(125, 125, 125));
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
		int distant = getLiadarDistant(frame, _result[i]);
		HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();
		if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)&& distant!=-1)
		{				
			Motorcyclist* motorcyclist = new Motorcyclist(tempFrame, _result[i], _headDetectReturnStruct, _rectangleColor, Scalar(0, 0, 255));
			#ifdef draw
				motorcyclist->DrawObj(frame);
				showLidarInformation(frame, _result[i], distant);
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
			#ifdef drawPredictDirect
			if (_type == ClassiferType::MotorbikeFrontBack)
			{
				putText(frame, motorcyclist->predictDirect(), CvPoint(_result[i].x + 20, _result[i].y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			}
			#endif	
			#endif			

			#ifdef UdpSocketServer
			_SentData->push_back(setSentData(_result[i], distant));						
			#endif // UdpSocketServer					
			_trackingObject.push_back(motorcyclist);
		}
	}	
}

void SvmClassifier::ClassifyCar(Mat & frame, Mat & grayFrame)
{
	_SentData->clear();
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	Mat tempFrame = frame.clone();
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(tempFrame);
		TrackingObject* tempCar = temp[i]->GetObject(ObjectType);
		
		int distant=-1; 
		if (!isOutOfRange(tempCar->getROI(), frame)) 
		{
			distant = getLiadarDistant(frame, tempCar->getROI());
		}
		
		if ((tempCar->confidence() > 0.3) &&(tempCar->missCount<4)/* && (distant !=-1)*/)
		{
#ifdef draw
			temp[i]->DrawObj(frame);	
			if (distant != -1) 
			{
				showLidarInformation(frame, tempCar->getROI(), distant);
			}
						
#ifdef drawImformation
			std::stringstream ss2;
			ss2 << tempCar->confidence();
			putText(frame, "t_hc", tempCar->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			putText(frame, ss2.str().substr(0, 4), CvPoint(tempCar->getROI().x, tempCar->getROI().y + 20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
#endif
#endif
#ifdef UdpSocketServer
			_SentData->push_back(setSentData(tempCar->getROI(), distant));			
#endif // UdpSocketServer			
			_trackingObject.push_back(temp[i]);
		}
#ifdef drawBye
		else
		{			
			temp[i]->DrawObj(frame);
			putText(frame, "bye", tempCar->getROI().tl(), 0, 1, Scalar(0, 0, 0), 1, 8, false);
			std::stringstream ss2;
			ss2 << tempCar->confidence();
			putText(frame, ss2.str().substr(0, 3), CvPoint(tempCar->getROI().x, tempCar->getROI().y + 20), 0, 1, Scalar(0, 0, 0), 1, 8, false);			
			std::stringstream ss4;
			ss4 << tempCar->missCount;
			putText(frame, ss4.str(), CvPoint(tempCar->getROI().x, tempCar->getROI().y + 40), 0, 1, Scalar(0, 0, 0), 1, 8, false);
			std::stringstream ss5;
			ss5 << distant;
			putText(frame, ss5.str(), CvPoint(tempCar->getROI().x, tempCar->getROI().y + 60), 0, 1, Scalar(0, 0, 0), 1, 8, false);
			if (isOutOfRange(tempCar->getROI(), frame)) 
			{				
				putText(frame, "out", CvPoint(tempCar->getROI().x, tempCar->getROI().y + 80), 0, 1, Scalar(0, 0, 0), 1, 8, false);
			}
		}
#endif
	}

	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);	
	refineROI(_result, _trackingObject);

	for (int i = 0; i<_result.size(); i++)
	{		
		//saveImage(tempFrame(_result[i]));
		int distant = getLiadarDistant(frame, _result[i]);		
		if (distant !=-1)
		{			
			DetectedCar* detectedCar = new DetectedCar(tempFrame, _result[i],_rectangleColor);
#ifdef draw
			detectedCar->DrawObj(frame);
			showLidarInformation(frame, _result[i], distant);			
			_trackingObject.push_back(detectedCar);
#ifdef drawImformation						
			putText(frame, "SVM", _result[i].tl(), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			/*std::stringstream ss;
			ss << foundweight[i];
			string temp ="w:"+ ss.str();
			putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 25), 1, 8, false);*/
#endif			
#endif			

#ifdef UdpSocketServer			
			_SentData->push_back(setSentData(_result[i], distant));			
#endif // UdpSocketServer			
		}
	}		
}


void SvmClassifier::Update_track(Mat &frame)
{			
	_SentData->clear();
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
			int distant = getLiadarDistant(frame, tempMotorcyclist->getROI());
			if (distant ==-1 && (tempMotorcyclist->getROI().y + tempMotorcyclist->getROI().height) > frame.rows * 4 / 10 && tempMotorcyclist->getROI().height < frame.rows / 2)
			{
				continue;
			}
			HeadDetectReturnStruct* _headDetectReturnStruct = new HeadDetectReturnStruct();			
			Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
			if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectReturnStruct)) 
			{
				((Motorcyclist*)_trackingObject[i])->setHeadStruct(_headDetectReturnStruct->center, _headDetectReturnStruct->radius);
				#ifdef draw										
				#ifdef drawHelmet
				((Motorcyclist*)_trackingObject[i])->DrawObjHead(frame);
				#endif // drawhelmet
				#endif // drawhelmet				
			}
			
			#ifdef draw						
			_trackingObject[i]->DrawObj(frame);						
			showLidarInformation(frame, tempMotorcyclist->getROI(), distant);			
			#ifdef drawImformation						
			putText(frame, "t", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
			#endif	
			#ifdef drawPredictDirect
			if (_type == ClassiferType::MotorbikeFrontBack)
			{
				putText(frame, _trackingObject[i]->predictDirect(), CvPoint(tempMotorcyclist->getROI().x + 20, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			}			
			#endif		
			#endif		

			#ifdef UdpSocketServer
			_SentData->push_back(setSentData(tempMotorcyclist->getROI(), distant));
			#endif					
		}
		#ifdef drawBye
		else 
		{
			_trackingObject[i]->DrawObj(frame);
			putText(frame, "byebye_t", _trackingObject[i]->GetObject(ObjectType)->getROI().tl(), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss2;
			ss2 << _trackingObject[i]->GetObject(ObjectType)->confidence();
			putText(frame, ss2.str(), CvPoint(_trackingObject[i]->GetObject(ObjectType)->getROI().x, _trackingObject[i]->GetObject(ObjectType)->getROI().y + 20), 0, 1, Scalar(122, 100, 100), 1, 8, false);
		}
		#endif
	}
}

void SvmClassifier::Update_trackCar(Mat & frame)
{
	_SentData->clear();
	Mat tempFrame = frame.clone();	
	for (int i = 0; i<_trackingObject.size(); i++)
	{
		_trackingObject[i]->UpdateObj(tempFrame);

		if (!isOutOfRange(_trackingObject[i]->GetObject(ObjectType)->getROI(), frame))
		{
			int distant = getLiadarDistant(frame, _trackingObject[i]->GetObject(ObjectType)->getROI());						
#ifdef draw						
			_trackingObject[i]->DrawObj(frame);									
			if (distant != -1)
			{
				showLidarInformation(frame, _trackingObject[i]->GetObject(ObjectType)->getROI(), distant);
			}
			
#ifdef drawImformation						
			putText(frame, "t", _trackingObject[i]->GetObject(ObjectType)->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
#endif	
#endif		

#ifdef UdpSocketServer
			_SentData->push_back(setSentData(_result[i], distant));
#endif					
		}
#ifdef drawBye
		else
		{
			_trackingObject[i]->DrawObj(frame);
			putText(frame, "byebye_t", _trackingObject[i]->GetObject(ObjectType)->getROI().tl(), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss2;
			ss2 << _trackingObject[i]->GetObject(ObjectType)->confidence();
			putText(frame, ss2.str(), CvPoint(_trackingObject[i]->GetObject(ObjectType)->getROI().x, _trackingObject[i]->GetObject(ObjectType)->getROI().y + 20), 0, 1, Scalar(122, 100, 100), 1, 8, false);
		}
#endif
	}
}

vector<SentData>* SvmClassifier::getSentData()
{
	return 	_SentData;
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
		return -1;
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
	int distant = _fusionManager->RequestDistance(frame, roi) / 1000;
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
	vector<Rect> tempvector;
	for (int i = 0; i < result.size(); i++)
	{
		if (result[i].area() != 0)
		{
			tempvector.push_back(result[i]);
		}
	}
	result.clear();
	result = tempvector;
}

void SvmClassifier::saveImage(Mat frame,string type)
{	
	std::stringstream ss;
	switch (_type)
	{
	case MotorbikeFrontBack:			
		resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
		ss << _framecountForSave;				
		cv::imwrite("pic\\MotorbikeFrontBack\\" + ss.str() + ".jpg", frame);
		_framecountForSave++;
		break;
	case MotorbikeSide:
		resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
		ss << _framecountForSave;		
		cv::imwrite("pic\\MotorbikeSide\\" + ss.str() + ".jpg", frame);
		_framecountForSave++;
		break;
	case CarFrontBack:
		resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
		ss << _framecountForSave;
		cv::imwrite("pic\\CarFrontBack\\" + ss.str() + ".jpg", frame);
		_framecountForSave++;
		break;
	case Pedestrian:
		resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
		ss << _framecountForSave;
		cv::imwrite("pic\\Pedestrian\\" + ss.str() + ".jpg", frame);
		_framecountForSave++;
		break;
	case Helmet:
		cout << "save error" << endl;
		break;
	default:
		cout <<"save error"<< endl;
		break;
	}
}


SentData SvmClassifier::setSentData(Rect roi, int distant)
{
	SentData sentData;
	sentData.ROI_Left_Top_X = roi.x;
	sentData.ROI_Left_Top_Y = roi.y;
	sentData.ROI_Width = roi.width;
	sentData.ROI_Height = roi.height;
	sentData.Object_Distance = distant;
	sentData.Object_Type = ObjectType;	
	return sentData;
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
