#include "SvmClassifier.h"


/*!
* 初始化SVM分類器
* @param featureName 為string 類型，為讀入的SVM模型名稱
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param windowSize 為Size 類型，為窗口大小，與訓練時正樣本的SIZE相同
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, Size windowSize, float threshold) : Classifier(type, rectangleColor, threshold), WINDOW_SIZE(windowSize)
{
	_svm = new PrimalSVM(featureName);
	Size _winSize = windowSize;
	Size _blockSize = Size(CELL_SIZE.width * 2, CELL_SIZE.height * 2);
	Size _blockStride = CELL_SIZE;
	Size _cellSize = CELL_SIZE;
	int _nbins = 9;
	_descriptor = HOGDescriptor(windowSize, _blockSize, _blockStride, _cellSize, 9, 1, -1, HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);	
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);	
	_descriptor.setSVMDetector(hogVector);
	t1 = nullptr;
	//static vector<float> detector = HOGDescriptor::getDefaultPeopleDetector();		
}
SvmClassifier::~SvmClassifier()
{
	stop();
}

bool SvmClassifier::start(Mat & frame)
{
	if (!t1)
	{
		void (SvmClassifier::*myFunc)(Mat &frame) = &SvmClassifier::Classify;
		t1 = new std::thread(myFunc, this, std::ref(frame));
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

void SvmClassifier::Classify(Mat &frame)
{
	//_descriptor.detectMultiScale(frame, _resultROI, _hitThreshold, CELL_SIZE,Size(),1.05,2.0,false);		
	Size winStride= Size(8,8);//为滑动窗口每次移动的距离
	Size padding = Size(24,24);//图像扩充的大小
	double scale = 1.05;//比例系数，即滑动窗口每次增加的比例
	double finalThreshold = 2;//校正系数，当一个目标被多个窗口检测出来时，该参数此时就起了调节作用，
	bool useMeanshiftGrouping = false;
	_descriptor.detectMultiScale(frame, _resultROI, _hitThreshold, winStride, padding, scale, finalThreshold, useMeanshiftGrouping);
}


/*!
* 將ROI影像的物件偵測結果紀錄至_resultROI
* @param frame 為Mat 類型，為原始輸入影像
* @param roiList 為vector<cv::Rect> 類型，為切割過後的ROI影像
*/
//void SvmClassifier::Classify(Mat &frame, vector<Rect> &roiList)
//{
//	_resultROI.clear();
//	vector<Rect> resultList;
//	for (int i = 0; i < roiList.size(); i++)
//	{
//		//cout << i << endl;
//		//rectangle(frame, roiList[i], Scalar(255, 0, 255), 5, 8, 0);						
//		//if (roiList[i].width < WINDOW_SIZE.width || roiList[i].height < WINDOW_SIZE.height)
//		//{
//		//continue;
//		//}
//
//		//_descriptor.detectMultiScale(Mat(frame, roiList[i]), resultList, THRESHOLD, CELL_SIZE);		     		
//
//		_descriptor.detectMultiScale(frame, resultList, _hitThreshold, CELL_SIZE, Size(), 1.05, 2);
//		//_descriptor.detectMultiScale(frame, resultList);
//
//		for (int j = 0; j < resultList.size(); j++)
//		{
//			_resultROI.push_back(Rect(resultList[j].x + roiList[i].x, resultList[j].y + roiList[i].y, resultList[j].width, resultList[j].height));
//		}		
//	}
//	resultList.clear();
//}

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