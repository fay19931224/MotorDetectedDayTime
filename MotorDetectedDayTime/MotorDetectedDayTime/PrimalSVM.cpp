#include "PrimalSVM.h"

PrimalSVM::PrimalSVM(string featureName)
{
	svmNew = SVM::load(featureName);
	if (svmNew->empty())
	{
		std::cout << "load svm detector failed!!!" << std::endl;
		return;
	}	
}
void PrimalSVM::getSupportVector(std::vector<float> &support_vector) const
{
	Mat svecsmat = svmNew->getSupportVectors();

	int svdim = svmNew->getVarCount();	
	int numofsv = svecsmat.rows;
	 
	cv::Mat alphamat = cv::Mat::zeros(numofsv, svdim, CV_32F);
	cv::Mat svindex = cv::Mat::zeros(1, numofsv, CV_64F);
	 
	double rho = svmNew->getDecisionFunction(0, alphamat, svindex);
	
	alphamat.convertTo(alphamat, CV_32F);
	cv::Mat Result = -1 * alphamat * svecsmat;	
	
	for (int i = 0; i < svdim; ++i)
	{
		support_vector.push_back(Result.at<float>(0, i));		
	}
	support_vector.push_back(rho);
}
