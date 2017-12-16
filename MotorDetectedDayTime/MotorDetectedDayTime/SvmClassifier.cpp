#include "SvmClassifier.h"

//#define draw
//#define drawImformation
//#define drawPredictDirect
//#define drawBye
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
	_type = type;
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
	_type = type;
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

		int distant = getLiadarDistant(frame, tempMotorcyclist->getROI());

		if ((tempMotorcyclist->confidence() > 0.35)/*||tempMotorcyclist->detectionCount>3)/*&& (distant <=40)*/&&
			!isOutOfRange(tempMotorcyclist->getROI(),frame)&&!isOutOfRange(tempHead->getROI(), frame)&&
			tempMotorcyclist->getROI().y + tempMotorcyclist->getROI().height > frame.rows / 2 && tempMotorcyclist->getROI().height < frame.rows / 2)
		{			
			if ((tempMotorcyclist->getROI()&tempHead->getROI())!= tempHead->getROI())
			{
				Rect tempROI = checkROI(tempMotorcyclist->getROI(), grayFrame);				
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
				putText(frame, "t_hc", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(0, 0, 255), 1, 8, false);
				putText(frame, ss2.str().substr(0,4), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y+20), 0, 1, Scalar(0, 0, 255), 1, 8, false);
			#endif
			#ifdef drawPredictDirect
				if (_type == ClassiferType::MotorbikeFrontBack) 
				{
					putText(frame, temp[i]->predictDirect(), CvPoint(tempMotorcyclist->getROI().x + 20, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
				}
				
			#endif 

			_trackingObject.push_back(temp[i]);
		}
		#ifdef drawBye
		else
		{			
			int a=tempMotorcyclist->detectionCount ;
			bool b=isOutOfRange(tempMotorcyclist->getROI(), frame);
			bool c = isOutOfRange(tempHead->getROI(), frame);

			temp[i]->DrawObj(frame);
			putText(frame, "bye", tempMotorcyclist->getROI().tl(), 0, 1, Scalar(122, 100, 100), 1, 8, false);			
			std::stringstream ss2;
			ss2 << tempMotorcyclist->confidence();
			putText(frame, ss2.str().substr(0,3), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 20), 0, 1, Scalar(122, 100, 100), 1, 8, false);

			std::stringstream ss3;			
			ss3 << a;			
			putText(frame, ss3.str(), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 40), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss4;
			ss4 << b;			
			putText(frame, ss4.str(), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 60), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss5;
			ss5 << c;			
			putText(frame, ss5.str(), CvPoint(tempMotorcyclist->getROI().x, tempMotorcyclist->getROI().y + 80), 0, 1, Scalar(122, 100, 100), 1, 8, false);
		}
		#endif
	}
	
	_descriptor.detectMultiScale(grayFrame,_result, foundweight, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);
	refineROI(_result, _trackingObject);	
		
	for (int i = 0; i<_result.size(); i++)
	{						
		Rect tempROI= checkROI(_result[i], grayFrame);								
		#ifdef draw
			cv::rectangle(frame, tempROI, Scalar(0, 0, 0), 2);
		#endif
		saveImage(frame(_result[i]));
		int distant = getLiadarDistant(frame, _result[i]);
		if ( _result[i].y+ _result[i].height<=frame.rows/2&& _result[i].height >= frame.rows / 2)
		{
			continue;
		}		
		HeadSVMDetectReturnStruct =_headDetected->detectedHead(grayFrame, tempROI);
		if (HeadSVMDetectReturnStruct.isDetected&&(_result[i]& HeadSVMDetectReturnStruct.detectedRect).area()!=0/*&& distant<=40*/)
		{								
			Motorcyclist* motorcyclist = new Motorcyclist(tempFrame, _result[i], HeadSVMDetectReturnStruct.detectedRect, _rectangleColor, Scalar(0, 0, 255));
			#ifdef draw
				motorcyclist->DrawObj(frame);
			#endif
			#ifdef drawImformation			
				if (distant != -1)
				{
					showLidarInformation(frame, _result[i], distant);
				}
				putText(frame, "SVM", _result[i].tl(), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			#endif			
			#ifdef drawPredictDirect
				if (_type == ClassiferType::MotorbikeFrontBack) 
				{
					putText(frame, motorcyclist->predictDirect(), CvPoint(_result[i].x + 20, _result[i].y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
				}
			#endif	
			/*std::stringstream ss;
			ss << foundweight[i];
			string temp ="w:"+ ss.str();
			putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 25), 1, 8, false);*/
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
		
		if (!isOutOfRange(_trackingObject[i]->GetObject("motorcyclist")->getROI(), frame))
		{
			#ifdef draw
				_trackingObject[i]->DrawObj(frame);
			#endif	
			#ifdef drawImformation
			int distant = getLiadarDistant(frame, _trackingObject[i]->GetObject("motorcyclist")->getROI());
			if (distant != -1)
			{
				showLidarInformation(frame, _trackingObject[i]->GetObject("motorcyclist")->getROI(), distant);
			}
			putText(frame, "t", _trackingObject[i]->GetObject("motorcyclist")->getROI().tl(), 0, 1, Scalar(0, 122, 255), 1, 8, false);
			#endif		
			#ifdef drawPredictDirect
			if (_type == ClassiferType::MotorbikeFrontBack) 
			{
				putText(frame, _trackingObject[i]->predictDirect(), CvPoint(_trackingObject[i]->GetObject("motorcyclist")->getROI().x + 20, _trackingObject[i]->GetObject("motorcyclist")->getROI().y + 20), 0, 1, Scalar(255, 122, 255), 1, 8, false);
			}				
			#endif	
		}
		#ifdef drawBye
		else 
		{
			_trackingObject[i]->DrawObj(frame);
			putText(frame, "byebye_t", _trackingObject[i]->GetObject("motorcyclist")->getROI().tl(), 0, 1, Scalar(122, 100, 100), 1, 8, false);
			std::stringstream ss2;
			ss2 << _trackingObject[i]->GetObject("motorcyclist")->confidence();
			putText(frame, ss2.str(), CvPoint(_trackingObject[i]->GetObject("motorcyclist")->getROI().x, _trackingObject[i]->GetObject("motorcyclist")->getROI().y + 20), 0, 1, Scalar(122, 100, 100), 1, 8, false);
		}
		#endif
	}
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
	if (temp.x - temp.width / 2 >= 0)
	{
		roi.x -= roi.width / 2;
	}
	if (temp.x + temp.width * 3 / 2 < frame.cols)
	{
		roi.width += roi.width / 2;
	}
	return _fusionManager->RequestDistance(frame, roi) / 1000;	
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
				if ((intersectionArea / resultIArea)>0.6|| (intersectionArea / resultJArea)>0.6)
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
	ss2 << _framecountForSave;
	string name = "pic\\"+ss.str()+"\\"+ ss2.str() + ".jpg";			
	resize(frame, frame, _svmDetectParameter.WINDOW_SIZE);
	cv::imwrite(name, frame);
	_framecountForSave++;
}