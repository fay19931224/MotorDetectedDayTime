#include "Classifier.h"

/*!
* �]�w�����������H�ήت��C��
* @param type ��ClassiferType �����A������������
* @param rectangleColor ��Scalar �����A����������������ɨϥΪ��ت��C��
* @param threshold ��float �����A�����������e�ȡA�ȶV�C�N�V�e�P�A�ȶV���N�V�Y��
*/
Classifier::Classifier(ClassiferType type, Scalar rectangleColor, float threshold) :_hitThreshold(threshold)
{
	_type = type;
	_rectangleColor = rectangleColor;	
}

Classifier::~Classifier()
{
}

/*!
* ���otracker���Q�R�����ϰ�A�ðO���Ӱ϶���index�A����o���϶���index�P��eframe��
* �t�Z��]�w�d��ɡA�R���Ӱ϶��C
*/
void Classifier::setRestROI()
{
	_restroi = new RestROI;
	if (_tracker._restTrackingObjs.size() > 0)
	{
		for (int i = 0; i < _tracker._restTrackingObjs.size(); i++)
		{
			_restroi->_trackingroi.push_back(_tracker._restTrackingObjs[i]);
		}
		_restroi->_index = _count;
		_recordlist.push_back(_restroi);

		_tracker._restTrackingObjs.clear();
	}
	for (int j = 0; j < _recordlist.size(); j++)
	{
		if (_count - _recordlist[j]->_index >= 3)
		{
			RestROI* temp = _recordlist[j];
			cout << "delete ROI! " << temp->_index << endl;
			_recordlist.erase(find(_recordlist.begin(), _recordlist.end(), temp));
			delete temp;
		}
	}
	_count++;
}

/*!
* �ˬdtracker���O�_���Q�R�����ϰ�
*/
bool Classifier::IsRestROI()
{
	if (_recordlist.size()>0)return true;
	else return false;
}

/*!
* �N�b���������Q�R�����l�ܰϰ�^��
*/
vector<Classifier::RestROI*> Classifier::getRestROI()
{
	return _recordlist;
}

/*!
* Tracker�i���s����ܩ��J�v���W
* @param frame ��Mat���A�A����J���v��
*/
bool Classifier::Update(Mat &frame)
{	
	//bool track = _tracker.update(frame, _resultROI);
	bool track = _tracker.updateTest(frame, _resultROI, _rectangleColor);
	
	_tracker.draw(frame, _rectangleColor);	
	
	return track;
}


/*!
* Tracker�i���s����ܩ��J�v���W
* @param frame ��Mat���A�A����J���v��
* @param fusionManager ��FusionManager���A�A�z�LfusionManager���oROI�d��Lidar��ƨӭp��Z��
* @param lidarDistanceData ��vector<long>���A�A����Jframe��Lidar�Z�����
* @param roiList ��vector<cv::Rect>���A�A�����ιL�᪺ROI�v��
*/
/*bool Classifier::Update(Mat &frame, FusionManager& fusionManager, vector<long>& lidarDistanceData, vector<Rect> &roiList)
{
	//_tracker.update(frame, _resultROI);
	bool track = _tracker.update(frame, _resultROI);
	_tracker.draw(frame, _rectangleColor, fusionManager, lidarDistanceData, roiList, _type);
	return track;
}
*/
