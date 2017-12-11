#include "SvmClassifier.h"

#define draw
#define drawImformation

/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor,svmDetectParameter svmDetectParameter, HeadSVMDetecter* headdetectd, FusionManager* fusionManager) : Classifier(type, rectangleColor)
{			
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
}


SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter) : Classifier(type, rectangleColor)
{
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
}


SvmClassifier::~SvmClassifier()
{
	stop();
}

bool SvmClassifier::start(Mat &frame,Mat & grayFrame)
{
	if (!t1)
	{
		void (SvmClassifier::*myFunc)(Mat &frame,Mat &grayFrame) = &SvmClassifier::Classify;
		t1 = new std::thread(myFunc, this, std::ref(frame),std::ref(grayFrame));
	}
	return t1 != nullptr;
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
	vector<Motorcyclist*> temp = _trackingObject;
	_trackingObject.clear();	
	Mat tempFrame = frame;
	for (int i = 0; i < temp.size(); i++)
	{		
		temp[i]->UpdateObj(tempFrame);
		TrackingObject* tempMotorcyclist = temp[i]->GetObject("motorcyclist");
		TrackingObject* tempHead = temp[i]->GetObject("head");
		if (((tempMotorcyclist->confidence() > 0.35)||tempMotorcyclist->detectionCount>3)&&
			!isOutOfRange(tempMotorcyclist->getROI(),frame)&&!isOutOfRange(tempHead->getROI(), frame))			
		{
			if ((tempMotorcyclist->getROI()&tempHead->getROI())!= tempHead->getROI())
			{
				Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);
				#ifdef draw
					//cv::rectangle(frame, tempROI, Scalar(125, 126, 122));
				#endif				
				HeadSVMDetectReturnStruct = _headDetected->detectedHead(grayFrame, tempROI);
				if (HeadSVMDetectReturnStruct.isDetected) 
				{
					tempHead->updateROI(HeadSVMDetectReturnStruct.detectedRect);
				}
				else
				{
					continue;
				}
			}
			#ifdef draw
				temp[i]->DrawObj(frame);
			#endif
			#ifdef drawImformation
				std::stringstream ss2;
				ss2 << tempMotorcyclist->confidence();
				putText(frame, temp[i]->predictDirect(), CvPoint(tempMotorcyclist->getROI().x + 20, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
				putText(frame, "t_hc", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
				putText(frame, ss2.str().substr(0,4), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y+20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			#endif
			_trackingObject.push_back(temp[i]);
		}
		/*else {
			temp[i]->DrawObj(frame, temp[i]->_color);
			putText(frame, "byebye", CvPoint(temp[i]->getROI().x, temp[i]->getROI().y), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss2;
			ss2 << temp[i]->confidence();
			putText(frame, ss2.str(), CvPoint(temp[i]->getROI().x, temp[i]->getROI().y+20), 0, 1, Scalar(122, 100, 100), 1, 8, false);
		}*/
	}
	
	_descriptor.detectMultiScale(grayFrame,_result, foundweight, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);
	refineROI(_result, _trackingObject);	
		
	for (int i = 0; i<_result.size(); i++)
	{						
		Rect tempROI= checkROI(_result[i], grayFrame);								
		#ifdef draw
			//cv::rectangle(frame, tempROI, Scalar(0, 0, 0), 2);
		#endif
		HeadSVMDetectReturnStruct =_headDetected->detectedHead(grayFrame, tempROI);
		if (HeadSVMDetectReturnStruct.isDetected&&(_result[i]& HeadSVMDetectReturnStruct.detectedRect).area()!=0&& foundweight[i]>2)
		{												
			showLidarInformation(frame,_result[i] );			
			Motorcyclist* motorcyclist = new Motorcyclist(tempFrame, _result[i], HeadSVMDetectReturnStruct.detectedRect, _rectangleColor, Scalar(0, 0, 255));
			#ifdef draw
				motorcyclist->DrawObj(frame);
			#endif
			#ifdef drawImformation				
				
				putText(frame, motorcyclist->predictDirect(), CvPoint(_result[i].x+20, _result[i].y+20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
				putText(frame, "SVM", _result[i].tl(), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			#endif			
			std::stringstream ss;
			ss << foundweight[i];
			string temp ="w:"+ ss.str();
			putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 25), 1, 8, false);
			_trackingObject.push_back(motorcyclist);
		}
	}
}

bool SvmClassifier::startUpdateTrack(Mat & frame)
{
	if (!t2) {
		void (SvmClassifier::*myFunc)(Mat &frame) = &SvmClassifier::Update_track;
		t2 = new std::thread(myFunc, this, std::ref(frame));
	}
	return t2!=nullptr;
}

void SvmClassifier::Update_track(Mat &frame)
{		
	Mat tempFrame = frame;
	for (int i = 0; i<_trackingObject.size(); i++)
	{								
		_trackingObject[i]->UpdateObj(tempFrame);
		
		if(!isOutOfRange(_trackingObject[i]->GetObject("motorcyclist")->getROI(), frame)&&( _trackingObject[i]->GetObject("motorcyclist")->getROI()&_trackingObject[i]->GetObject("head")->getROI()).area()!=0)
		{
			#ifdef draw
				_trackingObject[i]->DrawObj(frame);
			#endif	
			#ifdef drawImformation
				showLidarInformation(frame, _trackingObject[i]->GetObject("motorcyclist")->getROI());
				putText(frame, _trackingObject[i]->predictDirect(), CvPoint(_trackingObject[i]->GetObject("motorcyclist")->getROI().x + 20, _trackingObject[i]->GetObject("motorcyclist")->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
				putText(frame, "t", _trackingObject[i]->GetObject("motorcyclist")->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
			#endif		
		}		
	}
}

bool SvmClassifier::headDetectedheadDetected(Mat & frame, Mat & grayFrame, Rect roi)
{		
	svmDetectParameter headDetectParameter{ Size(32, 32),Size(8,8),static_cast<float>(0),Size(8,8),Size(8,8),1.05,2,false };
	Mat temp = grayFrame(roi);		
	_descriptor.detectMultiScale(temp, _result, headDetectParameter.hitThreshold, headDetectParameter.winStride, headDetectParameter.padding, headDetectParameter.scale);
	#ifdef draw
		cv::rectangle(frame, roi, Scalar(0, 0, 0), 2);
	#endif // draw			
		
	for (int i = 0; i < _result.size(); i++)
	{
		_result[i].x += roi.x;
		_result[i].y += roi.y;
		#ifdef draw
			cv::rectangle(frame, _result[i], _rectangleColor, 2);
		#endif // draw					
	}
	return _result.size();		
	//return true;	
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
	return Rect(x, y, width, height/3);
}

bool SvmClassifier::isOutOfRange(Rect roi,Mat frame)
{		
	return (roi.x<0)|| (roi.y<0)|| (roi.x+roi.width>frame.cols)|| (roi.y + roi.height>frame.rows) ? true:false;
}

void SvmClassifier::showLidarInformation(Mat &frame,Rect &roi)
{
	if (!_fusionManager) {
		return;
	}
	int distance = _fusionManager->RequestDistance(frame, roi)/1000;
	std::stringstream ss;
	ss << distance;
	string temp = "D:" + ss.str();
	putText(frame, temp, CvPoint(roi.x,roi.y+roi.height-20), 0, 1, Scalar(0, 122, 255), 1, 8, false);
}

/*
result要濾除重複的框
result裡跟trackingObject一樣的框砍掉
*/
void SvmClassifier::refineROI(vector<Rect> &result, vector<Motorcyclist*>  &trackingObject)
{	
	for (int i = 0; i < result.size(); i++)
	{
		for (int j = 0; j < result.size(); j++)
		{
			if (i != j&&result[i].area() != 0 && result[j].area() != 0)
			{
				Rect intersection = result[i] & result[j];
				float resultIArea = static_cast<float>(result[i].area());
				float resultJArea = static_cast<float>(result[j].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if ((intersectionArea / resultIArea)>0.5|| (intersectionArea / resultJArea)>0.5)
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
				Rect intersection = result[i] & trackingObject[j]->GetObject("motorcyclist")->getROI();
				float resultArea = static_cast<float>(result[i].area());
				float trackingObjectArea = static_cast<float>(trackingObject[j]->GetObject("motorcyclist")->getROI().area());
				float intersectionArea = static_cast<float>(intersection.area());
				if ((intersectionArea / resultArea)>0.6|| (intersectionArea / trackingObjectArea)>0.6)
				{					
					trackingObject[j]->SetObjectCount(1, "motorcyclist");
					result[i] = Rect(0, 0, 0, 0);					
				}
			}
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

void SvmClassifier::saveImage(Mat frame)
{	
	std::stringstream ss;
	ss << _svmDetectParameter.hitThreshold;	
	std::stringstream ss2;
	ss2 << i;	
	string name = "pic\\"+ss.str()+"\\"+ ss2.str() + ".jpg";			
	resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
	cv::imwrite(name, frame);
	i++;
}