#ifndef PRIMAL_SVM_H
#define PRIMAL_SVM_H

#include <vector>
#include <string>
#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace std;
using cv::Scalar;
using cv::Mat;
using cv::Rect;
using cv::Ptr;
using cv::ml::SVM;
class PrimalSVM
{
private:
	
public:
	Ptr<SVM> svmNew;
	PrimalSVM(string featureName);
	void getSupportVector(std::vector<float> &support_vector) const;	
};
#endif