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
	//vector<double> foundweight;
	//Size winStride = Size(8, 8);//为滑动窗口每次移动的距离
	//Size padding = Size(8, 8);//图像扩充的大小
	//float _hitThreshold;
	//double scale = 1.2;//比例系数，即滑动窗口每次增加的比例
	//double finalThreshold = 3;//校正系数，当一个目标被多个窗口检测出来时，该参数此时就起了调节作用，
	//bool useMeanshiftGrouping = false;
public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter);
	~SvmClassifier();	
	bool start(Mat &frame,Mat &grayFrame);
	bool stop();
	void Classify(Mat &frame,Mat &grayFrame);	
	void Update_track(Mat &frame);
};

#endif