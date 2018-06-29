#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H


#include "HeadDetecter.h"
#include "Classifier.h"
#include "PrimalSVM.h"
#include "Motorcyclist.h"
#include "DetectedCarAndPeople.h"
#include <thread>
#include <time.h>

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
	void refineROI(vector<Rect> &roiList, vector<DetectedObject*>  &trackroiList);	
	std::thread *t1 = nullptr;
	std::thread *t2 = nullptr;
	vector<DetectedObject*> _trackingObject;
	vector<double> foundweight;
	HogParameter _hogParameter;	
	HeadDetecter *_headDetected;
	Rect checkROI(Rect roi, Mat frame);
	bool isOutOfRange(Rect roi, Mat frame);	
	ClassiferType _type;
	string ObjectType;
	void setObjectType();
public:
	
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter);
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter, HeadDetecter* headdetectd);
	~SvmClassifier();	
	bool startClassify(Mat &frame,Mat &grayFrame);
	bool stop();
	void Classify(Mat &frame,Mat &grayFrame);			
	void ClassifyPedes(Mat &frame, Mat &grayFrame);
	bool startUpdateTrack(Mat &frame);
	void Update_track(Mat &frame);	
	void Update_trackPedes(Mat &frame);
		
	void ClassifyTest(Mat &frame, Mat &grayFrame);
};

#endif