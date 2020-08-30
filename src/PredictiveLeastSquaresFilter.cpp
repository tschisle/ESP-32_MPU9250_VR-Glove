
// Implementation of Trey Schisler's Predictive Least-Squares filter
// for low signal to noise ratio signals that require low computational demand
// and low latency (improved responsiveness)
// More information can be found on <treyschisler.com>

#include "PredictiveLeastSquaresFilter.h"

// There is a tradeoff in the samples parameter between high sensitivity and 
// high robustness. Future versions will include an optional auto sample set 
// based onUpdate call speed to approximate an optimal sample size. 
// Future updates might also perform a quick fourier transform to identify the
// common frequencies of noise.

//int samples; //how many averaged sample clusters used to find the slope
//float least_square_mat[4][samples]; //used to find the slope of the averaged sample clusters   1: sami - samavg 2: magi-magavg 3: 1*2 4: 1*1
//float least_square_avg; //used to find the slope of the averaged sample clusters
//float least_square_sum_comp[2]; //used to find the slope of the averaged sample clusters
//float least_square_slope_inter[2]; //holds slope and intercept values to approximate current value
//float approximation; //holds final approximated value
//float sammat[samples]; //holds the averaged samples in a matrix

void PLSF_Filter::PLSF_Initialization(){ 
//Precalculating static least square values HAS REDUNDANT ELEMENTS
	for (int _x = 0; _x < pls_samples; _x++) {
		least_square_mat[0][_x] = _x - ((pls_samples + 1) / 2);
		least_square_mat[3][_x] = least_square_mat[0][_x] * least_square_mat[0][_x];
		sammat[_x] = 0;
	}
	least_square_sum_comp[1] = 0;
	for (int _x = 0; _x < pls_samples; _x++) {
		least_square_sum_comp[1] = least_square_sum_comp[1] + least_square_mat[3][_x];
	}
}

float PLSF_Filter::PLSF_Update(float _input){	
	//clearing previous values and shifting data
    for (int x = 0; x < pls_samples; x++) {
        if (x < (pls_samples - 1)) {
            sammat[x] = sammat[x + 1];
        }
        least_square_avg = 0;
        least_square_sum_comp[0] = 0;
    }
    sammat[pls_samples - 1] = _input;
    //finding average mag values
    for (int x = 0; x < pls_samples; x++) {
        least_square_avg = least_square_avg + sammat[x];
    }
    least_square_avg = least_square_avg / pls_samples;
    //calculating least square values
    for (int x = 0; x < pls_samples; x++) {
        least_square_mat[1][x] = sammat[x] - least_square_avg;
        least_square_mat[2][x] = least_square_mat[0][x] * least_square_mat[1][x];
    }
    for (int x = 0; x < pls_samples; x++) {
        least_square_sum_comp[0] = least_square_sum_comp[0] + least_square_mat[2][x];
    }
    least_square_slope_inter[0] = least_square_sum_comp[0] / least_square_sum_comp[1]; //slopes
    least_square_slope_inter[1] = least_square_avg - (least_square_slope_inter[0] * ((pls_samples + 1) / 2)); //intercept
    //approximating current mag data from best fit LINE and reporting movement percentage
    approximation = (least_square_slope_inter[0] * pls_samples) + least_square_slope_inter[1];
    //this step provides the unitless value
	return(approximation);
}

void PLSF_Filter::PLSF_Clear(){
//Precalculating static least square values HAS REDUNDANT ELEMENTS
	for (int _x = 0; _x < pls_samples; _x++) {
		least_square_mat[1][_x] = 0;
		least_square_mat[2][_x] = 0;
		sammat[_x] = 0;
	}
	least_square_sum_comp[0] = 0;
	least_square_sum_comp[1] = 0;
	least_square_avg = 0;
	least_square_sum_comp[0] = 0;
	least_square_sum_comp[1] = 0;
	least_square_slope_inter[0] = 0;
	least_square_slope_inter[1] = 0;
}