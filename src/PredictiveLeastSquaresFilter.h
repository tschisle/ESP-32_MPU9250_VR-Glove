#ifndef _PredictiveLeastSquaresFilter_H_
#define _PredictiveLeastSquaresFilter_H_

#include <Arduino.h>


class PLSF_Filter
{
	_
	public:
	void PLSF_Initialization(uint8_t );

	float PLSF_Update(float );
							  
	void PLSF_Clear();
};

#endif 
