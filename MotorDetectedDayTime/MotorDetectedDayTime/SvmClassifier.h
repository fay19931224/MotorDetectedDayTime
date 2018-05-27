#ifndef SVM_CLASSIFIER_H
#define SVM_CLASSIFIER_H

#include "HeadDetecter.h"
#include "Classifier.h"
#include "PrimalSVM.h"
#include "Motorcyclist.h"
#include "DetectedCarAndPeople.h"
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
	void refineROI(vector<Rect> &roiList, vector<DetectedObject*>  &trackroiList);
	std::thread *t1;
	std::thread *t2;	
	vector<DetectedObject*> _trackingObject;
	vector<double> foundweight;
	HogParameter _hogParameter;
	
	HeadDetecter *_headDetected;
	int _framecountForSave = 0;	
	int _framecountForSavehead = 0;
	Rect checkROI(Rect roi, Mat frame);
	bool isOutOfRange(Rect roi, Mat frame);	
	void showLidarInformation(Mat &frame, Rect &roi, int distant);
	FusionManager *_fusionManager;
	int getLiadarDistant(Mat frame, Rect roi);
	ClassiferType _type;
	
	bool isClassifing = false;
	
	string ObjectType;
	void setObjectType();
public:
	
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter, FusionManager* fusionManager);
	SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, HogParameter hogParameter, FusionManager* fusionManager, HeadDetecter* headdetectd);
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