#include "Classifier.h"

/*!
* �]�w�����������H�ήت��C��
* @param type ��ClassiferType �����A������������
* @param rectangleColor ��Scalar �����A����������������ɨϥΪ��ت��C��
* @param threshold ��float �����A�����������e�ȡA�ȶV�C�N�V�e�P�A�ȶV���N�V�Y��
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
* Tracker�i���s����ܩ��J�v���W
* @param frame ��Mat���A�A����J���v��
*/
bool Classifier::Update(Mat &frame)
{	
	return 0;
}

