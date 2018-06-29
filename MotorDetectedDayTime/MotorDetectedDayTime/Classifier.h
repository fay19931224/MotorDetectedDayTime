#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <vector>
#include "ClassifierType.h"
#include "ObjManager.h"

using cv::Scalar;
using cv::Mat;
using cv::Rect;
using namespace std;

class Classifier
{

private:
	
	ClassiferType _type;	

public:
	Classifier(ClassiferType type, Scalar rectangleColor);
	virtual ~Classifier();

	virtual void Classify(Mat &frame);	
	virtual bool Update(Mat &frame);	

protected:	
	Scalar _rectangleColor;
	vector<Rect> _result;	
};

#endif