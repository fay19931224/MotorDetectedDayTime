#ifndef MODE_H
#define MODE_H

#include "Classifier.h"

/*!
* 此class可用來切換操作模式
*/
class Mode
{
private:

protected:
	//FusionManager _fusionManager;
	vector<Classifier*> _classifierList;

public:
	Mode();
	virtual ~Mode();
	virtual void Run() = 0;
};

#endif 