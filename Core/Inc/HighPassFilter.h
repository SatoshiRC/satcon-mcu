/*
 * HighPassFIlter.h
 *
 *  Created on: Aug 29, 2024
 *      Author: ohya
 */

#ifndef INC_HIGHPASSFILTER_H_
#define INC_HIGHPASSFILTER_H_

#include <cmath>

class HighPassFilter {
public:
	HighPassFilter(float sampleRate, float cutOffFreq, float q);

	float filter(float input);
private:
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;

	float input2;
	float input1;
	float out1;
	float out2;
};

#endif /* INC_HIGHPASSFILTER_H_ */
