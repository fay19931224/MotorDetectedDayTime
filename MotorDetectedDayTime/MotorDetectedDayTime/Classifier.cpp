#include "Classifier.h"

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

bool Classifier::Update(Mat &frame)
{	
	return 0;
}

