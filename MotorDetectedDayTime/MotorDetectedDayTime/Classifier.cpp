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

/*!
* 取得tracker中被刪除的區域，並記錄該區塊的index，當取得的區塊的index與當前frame數
* 差距到設定範圍時，刪除該區塊。
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
* 檢查tracker中是否有被刪除的區域
*/
bool Classifier::IsRestROI()
{
	if (_recordlist.size()>0)return true;
	else return false;
}

/*!
* 將在分類器中被刪除的追蹤區域回傳
*/
vector<Classifier::RestROI*> Classifier::getRestROI()
{
	return _recordlist;
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
	 _tracker.update(frame, _result, _rectangleColor);	
	 _tracker.draw(frame, _rectangleColor);
	return 0;
}

