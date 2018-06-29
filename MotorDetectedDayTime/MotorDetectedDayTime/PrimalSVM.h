#ifndef PRIMAL_SVM_H
#define PRIMAL_SVM_H

#include <vector>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>

using namespace std;
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