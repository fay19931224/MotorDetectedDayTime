#include "SvmClassifier.h"




/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor,svmDetectParameter svmDetectParameter, SvmClassifier* headdetectd) : Classifier(type, rectangleColor)
{			
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
	return true;
}

void SvmClassifier::Classify(Mat &frame,Mat &grayFrame)
{	
	
	vector<TrackingObject*> temp= _trackingObject;
	_trackingObject.clear();	
	
	for (int i = 0; i < temp.size(); i++)
	{		
		temp[i]->ObjUpdate(frame);
		if (temp[i]->confidence() > 0.7) 
		{						
			temp[i]->DrawObj(frame, temp[i]->_color);			
			_trackingObject.push_back(temp[i]);
		}
	}
			
	_descriptor.detectMultiScale(grayFrame,_result, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale, _svmDetectParameter.finalThreshold, _svmDetectParameter.useMeanshiftGrouping);
	refineROI(_result, _trackingObject);

	for (int i = 0; i<_result.size(); i++)
	{				
		if (_headDetected->headDetectedheadDetected(frame, grayFrame, _result[i])) 
		{
			TrackingObject *trackingObject = new TrackingObject(frame, _result[i], i, _rectangleColor);
			trackingObject->isTracking = true;
			trackingObject->isNewDetection = true;
			trackingObject->DrawObj(frame, _rectangleColor);			
			//saveImage(frame(_result[i]));
			/*std::stringstream ss;
			ss << foundweight[i];
			string temp ="weight:"+ ss.str();
			putText(frame, temp, CvPoint(_result[i].x, _result[i].y), 0, 1, Scalar(0, 255, 0), 1, 8, false);*/
			_trackingObject.push_back(trackingObject);
		}
		
	}
}

void SvmClassifier::Update_track(Mat &frame)
{		
	for (int i = 0; i<_trackingObject.size(); i++)
	{				
		_trackingObject[i]->ObjUpdate(frame);
		_trackingObject[i]->DrawObj(frame, _trackingObject[i]->_color);		
		//saveImage(frame(_result[i]));
	}
}

bool SvmClassifier::headDetectedheadDetected(Mat & frame, Mat & grayFrame, Rect roi)
{
	checkROI(roi, grayFrame);
	Mat temp = grayFrame(roi);
	
	/*
	resize(temp, temp,Size(),0.8,0.8);
	vector<float> features;
	int FEATURESAMOUNT = 324;
	Mat forTesting=Mat(Size(1,324),CV_32FC1);
	Mat response = Mat(Size(1, 1), CV_32FC1);
	float tempResponse;
	bool firstGet = true;
	float maxResponse = 0.0;
	Rect maxResponsePosition;
	for (int y = 0; y + 31 < temp.size().height; y++)
	{
		for (int x = 0; x + 31 < temp.size().width; x++)
		{
			_descriptor.compute(cv::Mat(temp, cv::Rect(x, y, 32, 32)), features, cv::Size(0, 0), cv::Size(0, 0));
			for (int descriptorIndex = 0; descriptorIndex < FEATURESAMOUNT; descriptorIndex++)
				forTesting.ptr<float>(0)[descriptorIndex] = features[descriptorIndex];
			tempResponse = _svm->svmNew->predict(forTesting, response,cv::ml::StatModel::RAW_OUTPUT);
			std::cout << response.ptr<float>(0)[0] << std::endl;
			system("pause");
			if (tempResponse < 0)
			{
				if ((firstGet == true) || (abs(tempResponse) >  maxResponse))
				{
					maxResponse = abs(tempResponse);
					maxResponsePosition = cv::Rect(x, y, 32, 32);
					cv::rectangle(frame, maxResponsePosition, Scalar(0, 0, 0), 2);
				}
				if (firstGet == true)
					firstGet = false;
			}
		}
	}	
	return true;
	*/	
	_descriptor.detectMultiScale(temp, _result, _svmDetectParameter.hitThreshold, _svmDetectParameter.winStride, _svmDetectParameter.padding, _svmDetectParameter.scale);
	//cv::rectangle(frame, roi, Scalar(0, 0, 0), 2);	
	for (int i = 0; i < _result.size(); i++)
	{
		_result[i].x += roi.x;
		_result[i].y += roi.y;
		//cv::rectangle(frame, _result[i], _rectangleColor, 2);
	}
	//return _result.size();		
	return true;
	
}

void SvmClassifier::checkROI(Rect & roi,Mat frame)
{
	int x = roi.x;
	int y = roi.y;
	int width = roi.width;
	int height = roi.height;
	if (x - 10 >0) {
		x -= 10;
	}
	else {
		x = 0;
	}
	if (y - 20 > 0) {
		y -= 20;
	}
	else
	{
		y = 0;;
	}
	if (roi.x +width + 20 >frame.cols)
	{
		width= frame.cols- roi.x-1;
	}
	else {
		width+=20;
	}
	roi = Rect(x, y, width, height/2);
}

/*
result要濾除重複的框
result裡跟trackingObject一樣的框砍掉
*/
void SvmClassifier::refineROI(vector<Rect> &result, vector<TrackingObject*>  &trackingObject)
{	
	for (int i = 0; i < result.size(); i++)
	{
		for (int j = 0; j < result.size(); j++)
		{
			if (i != j&&result[i].area() != 0 && result[j].area() != 0)
			{
				Rect intersection = result[i] & result[j];
				float roiArea = static_cast<float>(result[i].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if ((intersectionArea / roiArea)>0.5)
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
				Rect intersection = result[i] & trackingObject[j]->getROI();
				float roiArea = static_cast<float>(result[i].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if (intersectionArea / roiArea>0.5)
				{					
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