#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <vector>
#include "ClassifierType.h"
#include "ObjManager.h"
//#include "FusionManager.h"

using cv::Scalar;
using cv::Mat;
using cv::Rect;
using namespace std;

/*!
* 此class可以用來調整classifier分類及更新畫面的方式
*/
class Classifier
{

private:
	
	ClassiferType _type;	

public:
	Classifier(ClassiferType type, Scalar rectangleColor);
	virtual ~Classifier();

	virtual void Classify(Mat &frame);	
	virtual bool Update(Mat &frame);	
	
	int _count = 0;

	class RestROI
	{
	public:
		vector<Rect> _trackingroi;
		int _index;
	};
	void setRestROI();
	vector<RestROI*> getRestROI();
	vector<RestROI*> _recordlist;
	bool IsRestROI();

protected:
	ObjManager _tracker;
	Scalar _rectangleColor;
	vector<Rect> _result;
	RestROI* _restroi;	
};

#endif