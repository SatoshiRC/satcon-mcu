/*
 * HighPassFIlter.cpp
 *
 *  Created on: Aug 29, 2024
 *      Author: ohya
 */

#include <HighPassFilter.h>

HighPassFilter::HighPassFilter(float sampleRate, float cutOffFreq, float q) {
	float omega = 2.0f * 3.14159265f * cutOffFreq/sampleRate;
	float alpha = sin(omega) / (2.0f * q);

	a0 =   1.0f + alpha;
	a1 =  (-2.0f * cos(omega))/a0;
	a2 =   (1.0f - alpha)/a0;
	b0 =  (1.0f + cos(omega)) / 2.0f / a0;
	b1 = -(1.0f + cos(omega)) / a0;
	b2 =  (1.0f + cos(omega)) / 2.0f / a0;
}

float HighPassFilter::filter(float input){
	float tmp = input*b0 + input1*b1 + input2*b2 - out1*a1 - out2*a2;
	input2 = input1;
	input1 = input;
	out2 = out1;
	out1 = tmp;
	return tmp;
}
