#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "Classifier.h"
#include "PrimalSVM.h"
#include <thread>
/*!
* ��class�Ψӳ]�w��������HOG��CELLSIZE�H�Ϊ�l��SVM�������A�ô��Ѥ�������k�C
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
	std::thread *t1;
public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor = Scalar(0, 255, 0), Size windowSize = Size(64, 128), float threshold = 1);
	~SvmClassifier();	
	bool start(Mat &frame);
	bool stop();
	void Classify(Mat &frame);	

};

#endif