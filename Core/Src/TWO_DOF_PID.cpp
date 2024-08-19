/*
 * TWO_DOF_PID.cpp
 *
 *  Created on: Jul 12, 2024
 *      Author: ohya
 */

#include "TWO_DOF_PID.h"

template<class T>
T TWO_DOF_PID<T>::controller(T reference, T state){
	float time = elapsedTimer->getTimeMS();
	float diffTime = time - elapsedTime;
	elapsedTime = time;

	T diff = state - befState;
	T error = state - reference;
	integral += error*diffTime/1000.0;

	T res = reference * param.ffGain;
	res += error * param.pGain;
	res += integral * param.iGain;
	res -= diff * param.dGain;

	const auto lowerLimit = param.lowerControlLimit;
	const auto upperLimit = param.upperControlLimit;
	T antiWindup = 0;
	if(lowerLimit != 1 && res < lowerLimit){
		antiWindup = res - lowerLimit;
		res = lowerLimit;
	}else if(upperLimit != -1 && res > upperLimit){
		antiWindup = res - upperLimit;
		res = upperLimit;
	}
	T tmp = antiWindup / param.pGain;
	if(std::isfinite(tmp)){
		integral -= tmp;
	}else{
		integral -= antiWindup;
	}

	return res;
}
template float TWO_DOF_PID<float>::controller(float,float);
template double TWO_DOF_PID<double>::controller(double,double);
