#include "OfflineMode.h"
#include <sstream>

/*!
* 取得影像，雷達名稱以及fusion的類型
* 根據fusion類型決定ROI距離以及起始偵測距離，將Lidar資料與影像資料進行校正
* 根據分類器類型載入對應的Model放進classifierList供偵測使用
* @param videlFileName 為string 類型，為要偵測的影片名稱
* @param lidarFileName 為string 類型，為要偵測的影片對應的Lidar資料名稱
* @param videlFileName 為FusionType 類型，為fusion的類型
*/

void saveImage(Mat frame, int i)
{
	std::stringstream ss;
	ss << i;
	std::cout << ss.str() << std::endl;
	string name = ss.str() + ".jpg";
	cv::imwrite(name, frame);
}

OfflineMode::OfflineMode(string videoFileName, FusionType type, int currentModelType)
{
	_type = type;
	_waitKeySec = 30;
	_waitKeyChoosen = currentModelType;
	_videoFileName = videoFileName;
	
	//_classifierList.push_back(new SvmClassifier("Features\\svmFeature1002.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(72, 88), static_cast<float>(1)));
	//_classifierList.push_back(new SvmClassifier("Features\\motorbikeFeature.xml", ClassiferType::Motorbike, Scalar(0, 0, 255), Size(40, 64), static_cast<float>(0.2)));
	//_classifierList.push_back(new SvmClassifier("Features\\motorrow_all.xml", ClassiferType::Motorbike, Scalar(0, 255, 0), Size(96, 104), static_cast<float>(0.6)));			
	//_classifierList.push_back(new SvmClassifier("Features\\motorrow_upperv2.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(160, 104), 2));
	
	//_classifierList.push_back(new SvmClassifier("Features\\沒負樣本\\側面半身C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(48, 72), static_cast<float>(0)));
	//_classifierList.push_back(new SvmClassifier("Features\\沒負樣本\\側面全身C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(72, 88), static_cast<float>(0)));	
	//_classifierList.push_back(new SvmClassifier("Features\\沒負樣本\\正面全身C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 255, 0), Size(48, 104), static_cast<float>(0)));
	//_classifierList.push_back(new SvmClassifier("Features\\沒負樣本\\背面C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 0, 255), Size(48, 104), static_cast<float>(0)));

	
	_classifierList.push_back(new SvmClassifier("Features\\側面C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(72, 88), static_cast<float>(1)));
	_classifierList.push_back(new SvmClassifier("Features\\正面C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 255, 0), Size(48, 104), static_cast<float>(1)));
	_classifierList.push_back(new SvmClassifier("Features\\背面C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 0, 255), Size(48, 104), static_cast<float>(1)));
	
}

OfflineMode::~OfflineMode()
{
}

int OfflineMode::GetChoiceVideoType()
{
	return _waitKeyChoosen;
}

bool OfflineMode::WaitKey()
{
	int currentType = waitKey(_waitKeySec);
	if ('1' <= currentType && currentType <= '6')
	{
		_waitKeyChoosen = currentType - '0';
		return true;
	}
	else if (currentType == 27)//ESC
	{
		_waitKeyChoosen = 0;
		return true;
	}
	return false;
}

/*!
* 對ROI進行調整，將超出邊框的部分設定回邊界
* @param frame 為Mat型態，為輸入的原始影像
* @param roi 為Rect型態，為要進行物件偵測的區域
*/
Rect OfflineMode::adjustROI(Mat frame, Rect roi)
{
	if (roi.x<0)
	{
		roi.x = 0;
	}
	if (roi.y<0)
	{
		roi.y = 0;
	}
	if (roi.x + roi.width>frame.cols)
	{
		roi.width = frame.cols - roi.x;
	}
	if (roi.y + roi.height> frame.rows)
	{
		roi.height = frame.rows - roi.y;
	}
	return roi;
}


/*!
* 使用推估出的roi進行物件偵測，當一個分類器無法偵測出結果時，則交由另一個分類器進行判斷
* @param frame 為Mat型態，為輸入的原始影像
* @param grayFrame 為Mat型態，原始影像灰階後的影像
*/
void OfflineMode::Detect(Mat &frame, Mat &grayFrame,int count)
{	
	if (count % 30 == 0) 
	//if (true)
	{
		for (int k = 0; k < _classifierList.size(); k++)
		{
			((SvmClassifier*)_classifierList[k])->start(grayFrame);
		}
		for (int k = 0; k < _classifierList.size(); k++)
		{
			((SvmClassifier*)_classifierList[k])->stop();
			_classifierList[k]->Update(frame);
			//_trackingObject.push_back();
		}
		
	}
	else 
	{

	}
	
}



/*!
* 1.使用lidar資料時，讀取影像及Lidar資料後，對影像中進行物件的偵測並顯示出偵測結果
* 2.僅使用影像資料時，對影像中進行霍夫圓偵測尋找車輪，進行ROI的推估後
* 對ROI區域進行物件的偵測並顯示出偵測結果
*/
void OfflineMode::Run()
{
	VideoReader* reader = new VideoReader(_videoFileName);
	reader->StartRead();
	int dataQuantity = reader->GetDataQuantity();
	
	VideoWriter writer;
	writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, reader->getVideoSize());	
	if (!writer.isOpened())
	{
		return;
	}	
	for (int i = 0; i < dataQuantity; i++)
	{
	
		Mat frame;
		Mat grayFrame;
		reader->RequestOneData(frame);
		/*Rect roi = Rect(0, frame.rows / 4, frame.cols, frame.rows * 6 / 12);
		frame = frame(roi);*/
		cvtColor(frame, grayFrame, CV_BGR2GRAY);				
		
		
		//resize(grayFrame, grayFrame,Size(grayFrame.cols, grayFrame.rows));
		
		Detect(frame, grayFrame,i);		
		imshow(_videoFileName, frame);
		//writer.write(frame);
		if (WaitKey())
		{
			break;
		}
	}
	destroyAllWindows();
}

void OnGrab(void *info)
{
	// 在影像一進來的時後加入計算 Frame Rate
	static clock_t       StartTime = clock();
	clock_t                 EndTime = clock();
	int                        dt = EndTime - StartTime;
	StartTime = EndTime;
	if (dt != 0)
	{
		cout << "Frame : " << 1000.0 / dt << endl;
	}
}
