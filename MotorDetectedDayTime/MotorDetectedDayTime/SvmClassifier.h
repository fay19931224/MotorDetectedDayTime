#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H


#include "HeadDetecter.h"
#include "Classifier.h"
#include "PrimalSVM.h"
#include "Motorcyclist.h"
#include "DetectedPeople.h"
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
	vector<Rect> Classify(Mat &frame,Mat &grayFrame);
	vector<Rect> ClassifyWithHelmet(Mat &frame, Mat &grayFrame);
	vector<Rect> ClassifyPedes(Mat &frame, Mat &grayFrame);
	
	bool startUpdateTrack(Mat &frame);
	void Update_track(Mat &frame);
	void Update_trackPedes(Mat & frame, vector<Rect>& object);
		
	float sum = 0;
	vector<Rect> ClassifyTest(Mat &frame, Mat &grayFrame);
};

#endif