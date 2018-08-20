#include "SvmClassifier.h"

#define draw
#define drawHelmet
//#define drawImformation
//#define drawFilterOut
//#define drawBye

SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter, HeadDetecter* headdetectd) : Classifier(type, rectangleColor)
{				
	_type = type;	
	setObjectType();	
	_headDetected=headdetectd;
	_hogParameter = hogParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.winSize;
	Size _blockSize = Size(_hogParameter.cellSize.width * 2, _hogParameter.cellSize.height * 2);
	Size _blockStride = _hogParameter.cellSize;
	Size _cellSize = _hogParameter.cellSize;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_hogParameter.winSize, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, true, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);		
	//_descriptor.setVersion(_hogParameter.version);
	//_descriptor.setCache(false);
	_descriptor.setCache(true);
	_descriptor.setSVMDetector(hogVector);		
	_descriptor.setHroizon( _hogParameter.horion);		
}

SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter) : Classifier(type, rectangleColor)
{	
	_type = type;
	setObjectType();
	_hogParameter = hogParameter;
	_svm = new PrimalSVM(featureName);
	Size _winSize = _hogParameter.winSize;
	Size _blockSize = Size(_hogParameter.cellSize.width * 2, _hogParameter.cellSize.height * 2);
	Size _blockStride = _hogParameter.cellSize;
	Size _cellSize = _hogParameter.cellSize;
	int _normalizeType = _hogParameter.normalizeType;
	int _nbins = 9;
	_descriptor = HOGDescriptor(_hogParameter.winSize, _blockSize, _blockStride, _cellSize, _nbins, 1, -1.0, _normalizeType, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	//_descriptor.setCache(false);
	_descriptor.setVersion(false);
	_descriptor.setSVMDetector(hogVector);		
	_descriptor.setHroizon( _hogParameter.horion);	
}



SvmClassifier::~SvmClassifier()
{
	stop();
}

bool SvmClassifier::startClassify(Mat &frame,Mat & grayFrame)
{	
	if (!t1)
	{
		/*void (SvmClassifier::*myFunc)(Mat &frame, Mat &grayFrame);
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
		
		t1 = new std::thread(myFunc, this, std::ref(frame), std::ref(grayFrame));		*/
	}
	return t1 != nullptr;
}

bool SvmClassifier::startUpdateTrack(Mat & frame)
{
	if (!t2)
	{
	/*	void (SvmClassifier::*myFunc)(Mat &frame);
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
		t2 = new std::thread(myFunc, this, std::ref(frame));*/
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

vector<Rect> SvmClassifier::Classify(Mat & frame, Mat & grayFrame)
{		
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	vector<Rect> object;
//	line(frame, cv::Point(0, _hogParameter.horion*frame.rows), cv::Point(frame.cols, _hogParameter.horion*frame.rows),_rectangleColor);
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(frame);
		TrackingObject* tempMotorcyclist = temp[i]->GetObject();	
		if (tempMotorcyclist->confidence() > 0.3&&!isOutOfRange(tempMotorcyclist->getROI(),frame))
		{
			
			object.push_back(tempMotorcyclist->getROI());

			//Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
			//HeadDetectStruct* _headDetectStruct = new HeadDetectStruct();
			////if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectStruct))
			//if (_headDetected->detectedHeadHOGSVM(grayFrame, tempROI, _headDetectStruct))
			//{		
			//}
			_trackingObject.push_back(temp[i]);
		}
	}

	for (int i = 0; i < _result.size(); i++)
	{
		for (int j = 0; j < _trackingObject.size(); j++)
		{
			TrackingObject *temp = _trackingObject[j]->GetObject();			
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
		//cv::rectangle(frame, tempROI, Scalar(0, 255, 255), 2);
		HeadDetectStruct* _headDetectStruct = new HeadDetectStruct();
		if (_headDetected->detectedHead(grayFrame, tempROI, _headDetectStruct))		
		{
			Motorcyclist* motorcyclist = new Motorcyclist(frame, _result[i], _headDetectStruct, _rectangleColor, Scalar(0, 0, 255));
#ifdef draw
			//motorcyclist->DrawObj(frame);
			object.push_back(_result[i]);
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
	return object;
}

vector<Rect> SvmClassifier::ClassifyPedes(Mat & frame, Mat & grayFrame)
{		
	vector<Rect> object;
	//unsigned long start = clock();	
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);
	//unsigned long end = clock();
	//printf("a time=%1.3f seconds\n", (end - start) / 1000.0);

	Update_trackPedes(frame,object);
	unsigned long end2 = clock();
	//printf("b time=%1.3f seconds\n", (end2 - end) / 1000.0);
	/*vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	
	for (int i = 0; i < temp.size(); i++)
	{
		temp[i]->UpdateObj(frame);
		TrackingObject* tempPed = temp[i]->GetObject(ObjectType);				
		if ((tempPed->confidence() > 0.2)&&(!isOutOfRange(tempPed->getROI(),frame)))
		{
#ifdef draw
			temp[i]->DrawObj(frame);					
#endif
			_trackingObject.push_back(temp[i]);
		}
	}	*/
	
	for (int i = 0; i < _result.size(); i++)
	{
		for (int j = 0; j < _trackingObject.size(); j++)
		{
			TrackingObject *temp = _trackingObject[j]->GetObject();
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
		//detectedPed->DrawObj(frame);
		object.push_back(_result[i]);
#endif	
		_trackingObject.push_back(detectedPed);
	}
	/*unsigned long end3 = clock();
	printf("c time=%1.3f seconds\n", (end3 - end2) / 1000.0);*/
	return object;
}

void SvmClassifier::Update_track(Mat &frame)
{				
	/*Mat grayFrame;
	cvtColor(frame, grayFrame, CV_BGR2GRAY);*/
	for (int i = 0; i<_trackingObject.size(); i++)
	{								
		_trackingObject[i]->UpdateObj(frame);
		TrackingObject* tempMotorcyclist = _trackingObject[i]->GetObject();		

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

void SvmClassifier::Update_trackPedes(Mat & frame,vector<Rect>& object)
{		
	vector<DetectedObject*> temp = _trackingObject;
	_trackingObject.clear();
	for (int i = 0; i<temp.size(); i++)
	{
		if (temp[i]->isduplicate) 
		{
			continue;
		}
		temp[i]->UpdateObj(frame);
		TrackingObject* tempPed = temp[i]->GetObject();
		if ((tempPed->confidence() > 0.3) && (!isOutOfRange(tempPed->getROI(), frame))&& !temp[i]->isduplicate)
		{										
			temp[i]->isduplicate = false;
			_trackingObject.push_back(temp[i]);
		}
	}

	for (int i = 0; i < _trackingObject.size(); i++)
	{
		if (_trackingObject[i]->isduplicate)
		{
			continue;
		}
		for (int j = i+1; j < _trackingObject.size(); j++)
		{
			TrackingObject *tempI = _trackingObject[i]->GetObject();
			TrackingObject *tempJ = _trackingObject[j]->GetObject();
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
	
#ifdef draw						
	for (int i = 0; i<temp.size(); i++)
	{
		if (!_trackingObject[i]->isduplicate) 
		{
			_trackingObject[i]->DrawObj(frame);
			//object.push_back(_trackingObject[i]->GetObject(ObjectType)->getROI());
		}									
	}	
#endif	
}

vector<Rect> SvmClassifier::ClassifyTest(Mat & frame, Mat & grayFrame)
{		
	vector<Rect> object;
	//static int  count = 0;
	unsigned long start = clock();	
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	//_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, 1, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);
	unsigned long end = clock();
	//sum= ((end - start) / 1000.0);
	printf("total time=%1.3f seconds\n", (end - start) / 1000.0);
	//std::cin.get();
	for (int i = 0;i< _result.size(); i++)
	{
	/*	std::stringstream ss;
		ss << count++;*/
		/*Mat temp;
		resize(frame(_result[i]), temp, Size(72, 88));
		cv::imwrite("pic\\GH010109\\"+ss.str()+".jpg", temp);*/
		object.push_back(_result[i]);
		//cv::rectangle(frame, _result[i], _rectangleColor, 2);		
	}
	//printf("time=%1.3f\n", sum/2694.0);
	return object;
}

vector<Rect> SvmClassifier::ClassifyWithHelmet(Mat & frame, Mat & grayFrame)
{	
	vector<Rect> object;
	_descriptor.detectMultiScale(grayFrame, _result, foundweight, _hogParameter.hitThreshold, _hogParameter.winStride, _hogParameter.padding, _hogParameter.scale, _hogParameter.finalThreshold, _hogParameter.useMeanshiftGrouping);		
	for (int i = 0; i< _result.size(); i++)
	{
		if (_result[i].area() == 0 || (_result[i].y + _result[i].height) <= frame.rows * 4 / 10 || _result[i].height >= frame.rows / 2)
		{
			continue;
		}
		Rect tempROI = checkROI(_result[i], grayFrame);
		HeadDetectStruct* _headDetectStruct = new HeadDetectStruct();
		if (_headDetected->detectedHeadHoughCircles(grayFrame, tempROI, _headDetectStruct))
		{			
			object.push_back(_result[i]);			
		}		
	}
	return object;
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
		//height = (roi.height * 3) / 5;
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