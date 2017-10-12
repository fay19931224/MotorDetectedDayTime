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
	Ptr<SVM> svmNew;
public:
	PrimalSVM(string featureName);
	void getSupportVector(std::vector<float> &support_vector) const;
	//void getSupportVector(Mat &support_vector) const;
};
//class PrimalSVM : public cv::SVM
//{
//public:
//	void getSupportVector(std::vector<float> &support_vector) const
//	{
//		int sv_count = get_support_vector_count();//樣本數量
//		const CvSVMDecisionFunc* df = decision_func; // 表示SVM的决策函数or赋值决策函数指针
//		const double* alphas = df[0].alpha;
//		double rho = df[0].rho;
//		int var_count = get_var_count();// 样本的长度，也就是特征的维数必须匹配
//		support_vector.resize(var_count, 0);//初始化VECTOR，值皆為0
//		for (unsigned int r = 0; r < (unsigned)sv_count; r++)
//		{
//			float myalpha = alphas[r];
//			const float* v = get_support_vector(r);
//			for (int j = 0; j < var_count; j++, v++)
//			{
//				support_vector[j] += (-myalpha) * (*v);
//			}
//		}
//		support_vector.push_back(rho);
//	}
//};

#endif