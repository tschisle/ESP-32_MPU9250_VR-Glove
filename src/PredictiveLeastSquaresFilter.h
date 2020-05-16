/*
Notes:

This filter provides low latency smoothing for non-ideal signal to noise signals.
Currently, this library requires a "samples" definition that will determine how large
the matrices will be sized and also effects the initialization math. Additionally, 
since the number of samples effects the effectiveness of the filter, the value should
be fine-tuned to the situation.  A rule of thumb would be to have "samples" equal 
the number of samples taken in a quarter of the fastest expected input cycle.  

Stand by for equations and other helpful information.
*/

#ifndef _PredictiveLeastSquaresFilter_H_
#define _PredictiveLeastSquaresFilter_H_
#include <Arduino.h>
class PLSF_Filter
{
	//Variables/Matrices
	float least_square_mat[4][samples]; //used to find the slope of the averaged sample clusters   1: sami - samavg 2: magi-magavg 3: 1*2 4: 1*1
	float least_square_avg; //used to find the slope of the averaged sample clusters
	float least_square_sum_comp[2]; //used to find the slope of the averaged sample clusters
	float least_square_slope_inter[2]; //holds slope and intercept values to approximate current value
	float sammat[samples]; //holds the averaged samples in a matrix
	float approximation; //holds final approximated value
	public:
		void PLSF_Initialization(); //Initializes variables, matrices and does some early calculations to simplify the math for each update call
		float PLSF_Update(float ); //updates the buffer with the new input and outputs the filtered output
		void PLSF_Clear();  //clears the variables, matrices  (this function should be called before the first calibration step each time it's called
};
#endif 
