#include "Classifier.h"

/*!
* 設定分類器類型以及框的顏色
* @param type 為ClassiferType 類型，為分類器類型
* @param rectangleColor 為Scalar 類型，為分類器偵測物件時使用的框的顏色
* @param threshold 為float 類型，分類器的門檻值，值越低就越寬鬆，值越高就越嚴格
*/
Classifier::Classifier(ClassiferType type, Scalar rectangleColor)
{
	_type = type;
	_rectangleColor = rectangleColor;	
}

Classifier::~Classifier()
{
}


void Classifier::Classify(Mat & frame)
{
}

/*!
* Tracker進行更新並顯示於輸入影像上
* @param frame 為Mat型態，為輸入的影像
*/
bool Classifier::Update(Mat &frame)
{	
	return 0;
}

