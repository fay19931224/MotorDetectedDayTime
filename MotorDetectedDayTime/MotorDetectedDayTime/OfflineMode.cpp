#include "OfflineMode.h"
#include <sstream>

/*!
* ���o�v���A�p�F�W�٥H��fusion������
* �ھ�fusion�����M�wROI�Z���H�ΰ_�l�����Z���A�NLidar��ƻP�v����ƶi��ե�
* �ھڤ������������J������Model��iclassifierList�Ѱ����ϥ�
* @param videlFileName ��string �����A���n�������v���W��
* @param lidarFileName ��string �����A���n�������v��������Lidar��ƦW��
* @param videlFileName ��FusionType �����A��fusion������
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
	
	//_classifierList.push_back(new SvmClassifier("Features\\�S�t�˥�\\�����b��C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(48, 72), static_cast<float>(0)));
	//_classifierList.push_back(new SvmClassifier("Features\\�S�t�˥�\\��������C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(72, 88), static_cast<float>(0)));	
	//_classifierList.push_back(new SvmClassifier("Features\\�S�t�˥�\\��������C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 255, 0), Size(48, 104), static_cast<float>(0)));
	//_classifierList.push_back(new SvmClassifier("Features\\�S�t�˥�\\�I��C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 0, 255), Size(48, 104), static_cast<float>(0)));


	
	_classifierList.push_back(new SvmClassifier("Features\\����C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(255, 0, 0), Size(72, 88), static_cast<float>(0)));
	_classifierList.push_back(new SvmClassifier("Features\\����C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 255, 0), Size(48, 104), static_cast<float>(1)));
	_classifierList.push_back(new SvmClassifier("Features\\�I��C_SVC_LINEAR.xml", ClassiferType::Motorbike, Scalar(0, 0, 255), Size(48, 104), static_cast<float>(1)));
		
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
* ��ROI�i��վ�A�N�W�X��ت������]�w�^���
* @param frame ��Mat���A�A����J����l�v��
* @param roi ��Rect���A�A���n�i�檫�󰻴����ϰ�
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
* �ϥα����X��roi�i�檫�󰻴��A��@�Ӥ������L�k�����X���G�ɡA�h��ѥt�@�Ӥ������i��P�_
* @param frame ��Mat���A�A����J����l�v��
* @param grayFrame ��Mat���A�A��l�v���Ƕ��᪺�v��
*/
void OfflineMode::Detect(Mat &frame, Mat &grayFrame)
{
	//_classifierList[0]->Classify(grayFrame, _posibleROI);
	//if (_classifierList[0]->Update(frame) == 0)
	//{
	//	_classifierList[2]->setRestROI();
	//	vector<Rect> roi;
	//	if (_classifierList[2]->IsRestROI())
	//	{
	//		vector<Classifier::RestROI*> restroi = _classifierList[2]->getRestROI();
	//		for (int i = 0; i < restroi.size(); i++)
	//		{
	//			roi = restroi[i]->_trackingroi;
	//			for (int j = 0; j < roi.size(); j++)
	//			{
	//				roi[j] = adjustROI(frame, roi[j]);
	//				//rectangle(frame, Rect(roi[j].x, roi[j].y, roi[j].width, roi[j].height), Scalar(255, 255, 255));
	//			}
	//			_classifierList[1]->Classify(grayFrame, roi);
	//			if (_classifierList[1]->Update(frame) == 0)
	//			{
	//				_classifierList[0]->Classify(grayFrame, roi);
	//				_classifierList[0]->Update(frame);
	//			}
	//		}
	//	}
	//	else
	//	{
	//		for (int k = 0; k < _classifierList.size() - 1; k++)
	//		{
	//			_classifierList[k]->Classify(grayFrame);
	//			_classifierList[k]->Update(frame);
	//		}
	//	}
	//}
	

	for (int k = 0; k < _classifierList.size(); k++)
	{		
//		_classifierList[k]->Classify(grayFrame);
		((SvmClassifier*)_classifierList[k])->start(grayFrame);
		//_classifierList[k]->Update(frame);	
	}
	for (int k = 0; k < _classifierList.size(); k++)
	{
		((SvmClassifier*)_classifierList[k])->stop();
		_classifierList[k]->Update(frame);
	}
	//_posibleROI.clear();
}



/*!
* 1.�ϥ�lidar��ƮɡAŪ���v����Lidar��ƫ�A��v�����i�檫�󪺰�������ܥX�������G
* 2.�Ȩϥμv����ƮɡA��v�����i���N�Ҷ갻���M�䨮���A�i��ROI��������
* ��ROI�ϰ�i�檫�󪺰�������ܥX�������G
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
	double Time;
	double fps;
	for (int i = 0; i < dataQuantity; i++)
	{
		Time = cv::getTickCount();
		Mat frame;
		Mat grayFrame;
		reader->RequestOneData(frame);
		cvtColor(frame, grayFrame, CV_BGR2GRAY);		
		/*if (_type == FusionType::CarLeftSide || _type == FusionType::CarRightSide || _type == FusionType::CarFront)
		{
			_posibleROI.push_back(Rect(0, 0, grayFrame.cols - 1, grayFrame.rows - 1));			
		}*/
//		saveImage(frame, i);
		Detect(frame, grayFrame);
		/*fps = 1.0/(((double)cv::getTickCount() - Time) / cv::getTickFrequency());
		cout << "Frame : " << fps<<" "<< endl;
		std::stringstream ss;
		ss << fps;
		string name ="FPS:"+ ss.str();		
		cv::putText(frame, name, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,0,0));*/
		imshow(_videoFileName, frame);
		writer.write(frame);
		if (WaitKey())
		{
			break;
		}
	}
	destroyAllWindows();
}

void OnGrab(void *info)
{
	// �b�v���@�i�Ӫ��ɫ�[�J�p�� Frame Rate
	static clock_t       StartTime = clock();
	clock_t                 EndTime = clock();
	int                        dt = EndTime - StartTime;
	StartTime = EndTime;
	if (dt != 0)
	{
		cout << "Frame : " << 1000.0 / dt << endl;
	}
}
