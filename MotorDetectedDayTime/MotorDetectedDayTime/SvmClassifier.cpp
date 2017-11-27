#include "SvmClassifier.h"




/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor,svmDetectParameter svmDetectParameter) : Classifier(type, rectangleColor)
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
	return true;
}

void SvmClassifier::Classify(Mat &frame,Mat &grayFrame)
{	
	_trackingObject.clear();	
	vector<double> foundweight;
	_descriptor.detectMultiScale(grayFrame,_result, foundweight, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);
	/*
	    vector<int> v(10, 1);
    vector<int>::iterator it, end;
    const int val = 1;
    for(it=v.begin(); it!=v.end(); )
        if(*it==val) it = v.erase(it);
        else ++it;
	
	*/
	/*for (int i = 0; i<_result.size(); i++)
	{	
		Rect trackROI = _trackingObject[i]->getROI();				
		bool flag = true;
		for (int j = 0; j<_trackingObject.size(); j++)
		{	
			Rect temp = trackROI&_result[j];
			if (_trackingObject[j]->isTracking == false&& temp.area() != 0)
			{
				_trackingObject[j]->isTracking == true;
				flag = false;
				break;
			}			
		}
		if (flag) 
		{
			TrackingObject *trackingObject = new TrackingObject(frame, _result[i], i, _rectangleColor);
			trackingObject->isTracking = true;
			trackingObject->isNewDetection = true;
			trackingObject->DrawObj(frame, _rectangleColor);
			_trackingObject.push_back(trackingObject);
		}
		
	}
*/

	for (int i = 0; i<_result.size(); i++)
	{				
		TrackingObject *trackingObject = new TrackingObject(frame, _result[i], i, _rectangleColor);		
		trackingObject->isTracking = true;
		trackingObject->isNewDetection = true;
		trackingObject->DrawObj(frame, _rectangleColor);
		std::stringstream ss;
		ss << foundweight[i];		
		string temp ="weight:"+ ss.str();
		putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 0), 1, 8, false);
		_trackingObject.push_back(trackingObject);		
	}
}

void SvmClassifier::Update_track(Mat &frame)
{		
	for (int i = 0; i<_trackingObject.size(); i++)
	{				
		if (_trackingObject[i]->isTracking == true) 
		{
			_trackingObject[i]->ObjUpdate(frame);
			_trackingObject[i]->DrawObj(frame, _trackingObject[i]->_color);
			//_trackingObject[i]->isTracking = false;
		}		
	}
}

void SvmClassifier::refineROI(vector<Rect> &roiList)
{
	vector<Rect> tempvector;
	for (int i = 0; i < roiList.size(); i++)
	{
		for (int j = 0; j < roiList.size(); j++)
		{
			if (i != j&&roiList[i].area() != 0 && roiList[j].area() != 0)
			{
				Rect intersection = roiList[i] & roiList[j];
				float tempArea = static_cast<float>(roiList[i].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if (intersectionArea / tempArea>0.6)
				{
					roiList[i] = roiList[i] | roiList[j];
					roiList[j] = Rect(0, 0, 0, 0);
				}
			}
		}
	}
	for (int i = 0; i < roiList.size(); i++)
	{
		if (roiList[i].area() != 0)
		{
			tempvector.push_back(roiList[i]);
		}
	}
	roiList = tempvector;
}