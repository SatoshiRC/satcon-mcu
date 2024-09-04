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
	float diffTime = (time - elapsedTime)/1000.0;
	elapsedTime = time;

	T diff = state - befState;
	T error = reference - state;
//	integral += error*diffTime;

	return controller(reference,diff,error);
}
template float TWO_DOF_PID<float>::controller(float,float);
template double TWO_DOF_PID<double>::controller(double,double);

template<class T>
T TWO_DOF_PID<T>::controller(T reference, T Din, T Pin, T Iin){
	uint64_t time = elapsedTimer->getCount();
	float diffTime = (time - elapsedTime)/1000000.0;
	elapsedTime = time;

	if(integral + Iin*diffTime > param.upperControlLimit){
		integral = param.upperControlLimit / param.pGain;
	}else if(integral + Iin*diffTime < param.lowerControlLimit){
		integral = param.lowerControlLimit / param.pGain;
	}else{
		integral += Iin*diffTime;
	}

	if(std::isfinite(integral) == false){
		integral = 0;
	}

	T res = reference * param.ffGain;
	res += Pin * param.pGain;
	res += integral * param.iGain;
	res -= Din * param.dGain/diffTime;

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

	if(std::isfinite(integral) == false){
		integral = 0;
	}

	if(std::isfinite(res)==false){
		res = 0;
	}

	return res;
}
template float TWO_DOF_PID<float>::controller(float reference, float Din, float Pin, float Iin);

