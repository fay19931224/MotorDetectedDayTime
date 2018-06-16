#include "HeadDetecter.h"

HeadDetecter::HeadDetecter() 
{

}

HeadDetecter::HeadDetecter(string headSVM_XMLFilePath)
{
	svm = SVM::load(headSVM_XMLFilePath);
	if (svm->empty())
	{
		std::cout << "人頭SVM訓練檔讀取失敗!!!" << std::endl;
	}
	_hogDescriptor = new HOGDescriptor(WINDOW_SIZE, cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);	
}

HeadDetecter::~HeadDetecter()
{
}

void HeadDetecter::goDetectedHead(float scale, Mat temp, Mat image, Rect roi, HeadDetectReturnStruct &myReturn)
{
	std::vector<float> features;
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
			_hogDescriptor->compute(Mat(image, Rect(x, y, 32, 32)), features, Size(0, 0), Size(0, 0));
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

HeadDetectReturnStruct HeadDetecter::detectedHeadSVM(Mat &grayFrame, Rect roi)
{
	Mat temp = grayFrame(roi);	
	Mat image = grayFrame(roi);
	
	HeadDetectReturnStruct t1Return;
	HeadDetectReturnStruct t2Return;
	HeadDetectReturnStruct t3Return;
	HeadDetectReturnStruct t4Return;
	
	thread t1(&HeadDetecter::goDetectedHead,this,  0.6, temp, image, roi, std::ref(t1Return));
	thread t2(&HeadDetecter::goDetectedHead, this, 1.0, temp, image, roi, std::ref(t2Return));
	thread t3(&HeadDetecter::goDetectedHead, this, 1.4, temp, image, roi, std::ref(t3Return));
	thread t4(&HeadDetecter::goDetectedHead, this, 1.8, temp, image,  roi, std::ref(t4Return));
	
	t1.join();
	t2.join();
	t3.join();
	t4.join();
	HeadDetectReturnStruct IAmBig = t1Return;
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
	Rect tempRect = IAmBig.detectedRect;

//#pragma region rectAmplify
//	if (tempRect.x - 16>0)
//	{
//		IAmBig.detectedRect.x -= 16;
//	}
//	else
//	{
//		IAmBig.detectedRect.x = 0;
//	}
//	if (tempRect.y - 16>0)
//	{
//		IAmBig.detectedRect.y -= 16;
//	}
//	else
//	{
//		IAmBig.detectedRect.y = 0;
//	}
//	if (tempRect.x + tempRect.width + 16<grayFrame.cols)
//	{
//		IAmBig.detectedRect.width *= 2;
//	}
//	else
//	{
//		IAmBig.detectedRect.width = grayFrame.cols - tempRect.x - 1;
//	}
//	if (tempRect.y + tempRect.height + 16<grayFrame.rows)
//	{
//		IAmBig.detectedRect.height *= 2;
//	}
//	else
//	{
//		IAmBig.detectedRect.height = grayFrame.rows - tempRect.y - 1;
//	}
//#pragma endregion

	
	return IAmBig;
}

bool HeadDetecter::detectedHeadHoughCircles(Mat grayFrame, Rect roi, HeadDetectReturnStruct* headDetectReturnStruct)
{		
	if (roi.x + roi.width > grayFrame.cols) 
	{
		roi.width = grayFrame.cols - roi.x;
	}
	else if(roi.x<0)
	{
		roi.x = 0;		
	}
	Mat sample = grayFrame(roi);
	vector<cv::Vec3f> circles;
	double dp = 1;
	double minDist = sample.rows;
	double param1 = 100;
	double param2 = 15;
	int minRadius = 5;
	int maxRadius = sample.rows / 4;
	HoughCircles(sample, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
	if (circles.size() != 0)
	{
		cv::Point center(cvRound(circles[0][0])+roi.x, cvRound(circles[0][1])+roi.y);
		int radius = cvRound(circles[0][2]);		
		headDetectReturnStruct->center = center;
		headDetectReturnStruct->radius = radius;			
		return true;
	}
	else
	{
		Mat sample = grayFrame(roi);
		Mat dst;
		Laplacian(sample, dst, CV_8U, 1, 1, 0, cv::BORDER_DEFAULT);
		Mat abs_dst;
		convertScaleAbs(dst, abs_dst);		
		Mat add_dst;		
		addWeighted(sample, 1, abs_dst, 1, CV_8U, add_dst);
		double dp = 1;
		double minDist = sample.rows;
		double param1 = 50;
		double param2 = 15;
		int minRadius = 5;
		int maxRadius = sample.rows / 4;
		vector<cv::Vec3f> circles;
		HoughCircles(add_dst, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
		if (circles.size() != 0) 
		{			
			cv::Point center(cvRound(circles[0][0])+roi.x, cvRound(circles[0][1])+roi.y);
			int radius = cvRound(circles[0][2]);		
			headDetectReturnStruct->center = center;
			headDetectReturnStruct->radius = radius;
			return true;
		}
		else
		{
			//sobel??
			Mat sample = grayFrame(roi);
			Mat dst;
			Sobel(sample, dst, CV_8U, 1, 1);
			Mat abs_dst;
			convertScaleAbs(dst, abs_dst);
			Mat add_dst;			
			addWeighted(sample, 1, abs_dst, -1, CV_8U, add_dst);
			double dp = 1;
			double minDist = sample.rows;
			double param1 = 50;
			double param2 = 15;
			int minRadius = 5;
			int maxRadius = sample.rows / 4;
			vector<cv::Vec3f> circles;
			HoughCircles(add_dst, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
			if(circles.size()!=0)
			{
				cv::Point center(cvRound(circles[0][0]) + roi.x, cvRound(circles[0][1]) + roi.y);
				int radius = cvRound(circles[0][2]);				
				headDetectReturnStruct->center = center;
				headDetectReturnStruct->radius = radius;
				return true;
			}			
		}
	}
	return false;
}