#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "Classifier.h"
#include "PrimalSVM.h"
#include <thread>
/*!
* 此class用來設定分類器的HOG的CELLSIZE以及初始化SVM分類器，並提供分類的方法。
*/

using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Size;
using cv::HOGDescriptor;
class SvmClassifier : public Classifier
{
private:	
	
	HOGDescriptor _descriptor;
	PrimalSVM *_svm;
	void refineROI(vector<Rect> &roiList);
	std::thread *t1;
	vector<TrackingObject*> _trackingObject;
	svmDetectParameter _svmDetectParameter;	
	void saveImage(Mat frame);	
	SvmClassifier *_headDetected;
	int i = 0;
	bool headDetectedheadDetected(Mat & frame, Mat &grayFrame, Rect roi);
	void checkROI(Rect& roi, Mat frame);
public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter);
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter, SvmClassifier* headdetectd);
	~SvmClassifier();	
	bool start(Mat &frame,Mat &grayFrame);
	bool stop();
	void Classify(Mat &frame,Mat &grayFrame);	
	void Update_track(Mat &frame);
	
	
};

#endif