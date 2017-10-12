#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "Classifier.h"
#include "PrimalSVM.h"

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
	const Size CELL_SIZE = Size(8, 8);
	const Size WINDOW_SIZE;
	HOGDescriptor _descriptor;
	PrimalSVM *_svm;
	void refineROI(vector<Rect> &roiList);
public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor = Scalar(0, 255, 0), Size windowSize = Size(64, 128), float threshold = 1);
	~SvmClassifier();
	void Classify(Mat &frame);
	void Classify(Mat &frame, vector<Rect> &roiList);
};

#endif