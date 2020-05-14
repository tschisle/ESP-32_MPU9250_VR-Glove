#ifndef _PredictiveLeastSquaresFilter_H_
#define _PredictiveLeastSquaresFilter_H_

#include <Arduino.h>

void PLSF_Initialization(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);

float PLSF_Update(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
							  
void PLSF_Clear();

#endif 
