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
	ObjManager _tracker;
	ClassiferType _type;
	Scalar _rectangleColor;

public:
	Classifier(ClassiferType type, Scalar rectangleColor, float threshold);
	virtual ~Classifier();

	virtual void Classify(Mat &frame) = 0;
	virtual void Classify(Mat &frame, vector<Rect> &roiList) = 0;
	virtual bool Update(Mat &frame);
	//virtual bool Update(Mat &frame, FusionManager& fusionManager, vector<long>& lidarDistanceData, vector<Rect> &roiList);
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
	vector<Rect> _resultROI;
	RestROI* _restroi;
	const float THRESHOLD;
};

#endif