#include "HeadSVMDetecter.h"

HeadSVMDetecter::HeadSVMDetecter(string headSVM_XMLFilePath)
{
	svm = SVM::load(headSVM_XMLFilePath);
	if (svm->empty())
	{
		std::cout << "人頭SVM訓練檔讀取失敗!!!" << std::endl;
	}
	hogDescriptor = new HOGDescriptor(WINDOW_SIZE, cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
}

void HeadSVMDetecter::goDetectedHead(float scale, Mat temp, Mat image, Rect roi, HeadSVMDetectReturnStruct &myReturn)
{
	Mat forTesting = Mat(Size(FEATURESAMOUNT, 1), CV_32FC1);

	float tempResponse = 0;
	bool firstGet = true;
	float maxResponse = 0;
	Rect maxResponsePosition;
	float maxResponseScale = 0;

	cv::resize(temp, image, Size(0, 0), scale, scale);

	for (int y = 0; y + 31 < image.size().height; y+=4)
	{
		for (int x = 0; x + 31 < image.size().width; x+=4)
		{
			hogDescriptor->compute(Mat(image, Rect(x, y, 32, 32)), features, Size(0, 0), Size(0, 0));
			for (int descriptorIndex = 0; descriptorIndex < FEATURESAMOUNT; descriptorIndex++)
				forTesting.ptr<float>(0)[descriptorIndex] = features[descriptorIndex];
			tempResponse = svm->predict(forTesting, noArray(), StatModel::RAW_OUTPUT);

			if (tempResponse < 0)
			{
				if (firstGet == true || abs(tempResponse) > maxResponse)
				{
					maxResponseScale = scale;
					maxResponse = abs(tempResponse);
					maxResponsePosition = Rect(x, y, 32, 32);
				}
				firstGet = false;
			}
			features.clear();
		}
	}
	
	if (firstGet != true)
	{
		//偵測到
		maxResponsePosition.x /= maxResponseScale;
		maxResponsePosition.x += roi.x;
		maxResponsePosition.y /= maxResponseScale;
		maxResponsePosition.y += roi.y;
		maxResponsePosition.height /= maxResponseScale;
		maxResponsePosition.width /= maxResponseScale;

		myReturn.isDetected = true;
		myReturn.detectedRect = maxResponsePosition;
		myReturn.scale = maxResponse;
	}
	else
	{
		//沒找到
		myReturn.isDetected = false;
		myReturn.detectedRect = Rect(0, 0, 0, 0);
		myReturn.scale = 0.0;
	}
}

HeadSVMDetectReturnStruct HeadSVMDetecter::detectedHead(Mat &grayFrame, Rect roi)
{
	Mat temp = grayFrame(roi);
	Mat image = grayFrame(roi);
	
	HeadSVMDetectReturnStruct t1Return;
	HeadSVMDetectReturnStruct t2Return;
	HeadSVMDetectReturnStruct t3Return;
	HeadSVMDetectReturnStruct t4Return;
	
	thread t1(&HeadSVMDetecter::goDetectedHead,this,  0.6, temp, image, roi, std::ref(t1Return));
	thread t2(&HeadSVMDetecter::goDetectedHead, this, 1.0, temp, image, roi, std::ref(t2Return));
	thread t3(&HeadSVMDetecter::goDetectedHead, this, 1.4, temp, image, roi, std::ref(t3Return));
	thread t4(&HeadSVMDetecter::goDetectedHead, this, 1.8, temp, image,  roi, std::ref(t4Return));
	
	t1.join();
	t2.join();
	t3.join();
	t4.join();
	HeadSVMDetectReturnStruct IAmBig = t1Return;
	if (IAmBig.scale < t2Return.scale)
	{
		IAmBig = t2Return;
	}
	if (IAmBig.scale < t3Return.scale)
	{
		IAmBig = t3Return;
	}
	if (IAmBig.scale < t4Return.scale)
	{
		IAmBig = t4Return;
	}
	return IAmBig;
}

void HeadSVMDetecter::draw()
{
}

HeadSVMDetecter::~HeadSVMDetecter()
{
}