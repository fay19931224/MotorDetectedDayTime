#include "SvmClassifier.h"


/*!
* ��l��SVM������
* @param featureName ��string �����A��Ū�J��SVM�ҫ��W��
* @param type ��ClassiferType �����A������������
* @param rectangleColor ��Scalar �����A����������������ɨϥΪ��ت��C��
* @param windowSize ��Size �����A�����f�j�p�A�P�V�m�ɥ��˥���SIZE�ۦP
* @param threshold ��float �����A�����������e�ȡA�ȶV�C�N�V�e�P�A�ȶV���N�V�Y��
*/
SvmClassifier::SvmClassifier(string featureName, ClassiferType type, Scalar rectangleColor, Size windowSize, float threshold) : Classifier(type, rectangleColor, threshold), WINDOW_SIZE(windowSize)
{
	_svm = new PrimalSVM(featureName);
	Size _winSize = windowSize;
	Size _blockSize = Size(CELL_SIZE.width * 2, CELL_SIZE.height * 2);
	Size _blockStride = CELL_SIZE;
	Size _cellSize = CELL_SIZE;
	int _nbins = 9;
	_descriptor = HOGDescriptor(windowSize, Size(CELL_SIZE.width * 2, CELL_SIZE.height * 2), CELL_SIZE, CELL_SIZE, 9, 1, -1, HOGDescriptor::L2Hys, 0.2, false, HOGDescriptor::DEFAULT_NLEVELS, false);
	vector<float> hogVector;
	_svm->getSupportVector(hogVector);
	//static vector<float> detector = HOGDescriptor::getDefaultPeopleDetector();		
	_descriptor.setSVMDetector(hogVector);

}
SvmClassifier::~SvmClassifier()
{

}

void SvmClassifier::Classify(Mat &frame)
{
	_descriptor.detectMultiScale(frame, _resultROI, THRESHOLD, CELL_SIZE);
}


/*!
* �NROI�v�������󰻴����G������_resultROI
* @param frame ��Mat �����A����l��J�v��
* @param roiList ��vector<cv::Rect> �����A�����ιL�᪺ROI�v��
*/
void SvmClassifier::Classify(Mat &frame, vector<Rect> &roiList)
{
	_resultROI.clear();
	vector<Rect> resultList;
	for (int i = 0; i < roiList.size(); i++)
	{

		//cout << i << endl;
		//rectangle(frame, roiList[i], Scalar(255, 0, 255), 5, 8, 0);						
		/*if (roiList[i].width < WINDOW_SIZE.width || roiList[i].height < WINDOW_SIZE.height)
		{
		continue;
		}*/

		////_descriptor.detectMultiScale(Mat(frame, roiList[i]), resultList, THRESHOLD, CELL_SIZE);		     

		double hit_threshold = THRESHOLD;

		_descriptor.detectMultiScale(frame, resultList, hit_threshold, CELL_SIZE, Size(), 1.05, 2);



		//cout << "_descriptorsize:" << resultList.size() << endl;

		for (int j = 0; j < resultList.size(); j++)
		{
			_resultROI.push_back(Rect(resultList[j].x + roiList[i].x, resultList[j].y + roiList[i].y, resultList[j].width, resultList[j].height));
		}
		//cout << "_resultROI:" << _resultROI.size() << endl;
		//resultList.clear();
	}
	resultList.clear();
}

void SvmClassifier::refineROI(vector<Rect> &roiList)
{
	vector<Rect> tempvector;
	for (int i = 0; i < roiList.size(); i++)
	{
		for (int j = 0; j < roiList.size(); j++)
		{
			if (i != j&&roiList[i].area() != 0 && roiList[j].area() != 0)
			{
				Rect intersection = roiList[i] & roiList[j];
				float tempArea = static_cast<float>(roiList[i].area());
				float intersectionArea = static_cast<float>(intersection.area());
				if (intersectionArea / tempArea>0.6)
				{
					roiList[i] = roiList[i] | roiList[j];
					roiList[j] = Rect(0, 0, 0, 0);
				}
			}
		}
	}
	for (int i = 0; i < roiList.size(); i++)
	{
		if (roiList[i].area() != 0)
		{
			tempvector.push_back(roiList[i]);
		}
	}
	roiList = tempvector;
}