﻿#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "HeadSVMDetecter.h"
#include "Classifier.h"
#include "PrimalSVM.h"
#include "Motorcyclist.h"
#include <thread>
#include "FusionManager.h"

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
	void refineROI(vector<Rect> &roiList, vector<Motorcyclist*>  &trackroiList);
	std::thread *t1;
	std::thread *t2;	
	vector<Motorcyclist*> _trackingObject;
	vector<double> foundweight;
	svmDetectParameter _svmDetectParameter;	
	void saveImage(Mat frame);	
	HeadSVMDetecter *_headDetected;
	int _framecountForSave = 0;	
	Rect checkROI(Rect roi, Mat frame);
	bool isOutOfRange(Rect roi, Mat frame);
	HeadSVMDetectReturnStruct HeadSVMDetectReturnStruct;
	void showLidarInformation(Mat &frame, Rect &roi, int distant);
	FusionManager *_fusionManager;
	int getLiadarDistant(Mat frame, Rect roi);
	ClassiferType _type;
public:
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter);
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, svmDetectParameter svmDetectParameter, HeadSVMDetecter* headdetectd, FusionManager* fusionManager);
	~SvmClassifier();	
	bool start(Mat &frame,Mat &grayFrame);
	bool stop();
	void Classify(Mat &frame,Mat &grayFrame);	
	bool startUpdateTrack(Mat &frame);
	void Update_track(Mat &frame);	
};

#endif