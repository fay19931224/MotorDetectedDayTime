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

HeadSVMDetectReturnStruct HeadSVMDetecter::detectedHead(Mat &frame, Mat grayFrame, Rect roi)
{
	Mat temp = grayFrame(roi);
	Mat image = grayFrame(roi);
	Mat forTesting = Mat(Size(FEATURESAMOUNT, 1), CV_32FC1);
	
	float tempResponse = 0;
	bool firstGet = true;
	float maxResponse = 0;
	Rect maxResponsePosition;
	float maxResponseScale = 0;	

	for (float scale = 0.6; scale <= 1.8; scale = scale + 0.4)
	{
		cv::resize(temp, image, Size(0, 0), scale, scale);

		for (int y = 0; y + 31 < image.size().height; y++)
		{
			for (int x = 0; x + 31 < image.size().width; x++)
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
	}	
	_headSVMDetectReturnStruct.isError = false;
	if (firstGet != true)
	{		
		maxResponsePosition.x /= maxResponseScale;
		maxResponsePosition.x += roi.x;
		maxResponsePosition.y /= maxResponseScale;
		maxResponsePosition.y += roi.y;
		maxResponsePosition.height /= maxResponseScale;
		maxResponsePosition.width /= maxResponseScale;				
		
		_headSVMDetectReturnStruct.isDetected = true;
		_headSVMDetectReturnStruct.detectedRect = maxResponsePosition;
		_headSVMDetectReturnStruct.scale = maxResponseScale;
				
		/*std::stringstream ss2;
		ss2 << maxResponsePosition.x;
		string name = "pic\\" + ss2.str() + ".jpg";		
		cv::rectangle(temp, _headSVMDetectReturnStruct.detectedRect, Scalar(255, 255, 255), 2);		
		cv::imshow("t", temp);*/

		/*maxResponsePosition.x = roi.x+ maxResponsePosition.x;
		maxResponsePosition.y = roi.y+ maxResponsePosition.y;*/				
	}
	else
	{
		_headSVMDetectReturnStruct.isDetected = false;
		_headSVMDetectReturnStruct.detectedRect = Rect(0, 0, 0, 0);
		_headSVMDetectReturnStruct.scale = 0.0;
	}
	return _headSVMDetectReturnStruct;
}

void HeadSVMDetecter::draw()
{
}

HeadSVMDetecter::~HeadSVMDetecter()
{
}