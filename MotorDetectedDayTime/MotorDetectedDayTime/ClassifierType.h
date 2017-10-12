#ifndef CLASSIFIER_TYPE_H
#define CLASSIFIER_TYPE_H



/*!
* 此.h檔提供了使用的分類型態，以及與LIDAR結合時該使用哪組對應參數
*/
enum ClassiferType
{
	Vehicle,
	Pedestrian,
	Motorbike,
	UnKnown
};

enum FusionType
{
	CarFront,
	CarRightSide,
	CarLeftSide,
	CarBack
};

#endif;