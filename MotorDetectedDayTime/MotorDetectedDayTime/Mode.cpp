#include "Mode.h"

Mode::Mode()
{
}

Mode::~Mode()
{
	for (int i = 0; i < _classifierList.size(); i++)
	{
		cout << i << endl;
		//	delete _classifierList[i];
	}
}