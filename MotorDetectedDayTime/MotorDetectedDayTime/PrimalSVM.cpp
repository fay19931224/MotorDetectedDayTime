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
	int DescriptorDim = svmNew->getVarCount();
	
	//获取svecsmat，元素类型为float  
	Mat svecsmat = svmNew->getSupportVectors();
	//特征向量维数  
	int svdim = svmNew->getVarCount();	
	int numofsv = svecsmat.rows;

	//alphamat和svindex必须初始化，否则getDecisionFunction()函数会报错  
	cv::Mat alphamat = cv::Mat::zeros(numofsv, svdim, CV_32F);
	cv::Mat svindex = cv::Mat::zeros(1, numofsv, CV_64F);

	cv::Mat Result;
	double rho = svmNew->getDecisionFunction(0, alphamat, svindex);
	//将alphamat元素的数据类型重新转成CV_32F  
	alphamat.convertTo(alphamat, CV_32F);
	Result = -1 * alphamat * svecsmat;
	for (int i = 0; i < svdim; ++i)
	{
		support_vector.push_back(Result.at<float>(0, i));
	}
	support_vector.push_back(rho);

	//saving HOGDetectorForOpenCV.txt  
	/*std::ofstream fout("HOGDetectorForOpenCV.txt");
	for (int i = 0; i < support_vector.size(); ++i)
	{
	fout << support_vector[i] << std::endl;
	}*/
}
