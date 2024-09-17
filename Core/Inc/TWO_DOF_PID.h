/*
 * TWO_DOF_PID.h
 *
 *  Created on: Jul 12, 2024
 *      Author: ohya
 */

#ifndef INC_TWO_DOF_PID_H_
#define INC_TWO_DOF_PID_H_

#include <cmath>
#include "delta_time.h"

template<class T = float>
struct TWO_DOF_PID_PARAM{
	T ffGain;
	T pGain;
	T iGain;
	T dGain;
	T upperControlLimit;
	T lowerControlLimit;

	TWO_DOF_PID_PARAM(T ffGain = 0,
	T pGain = 0, T iGain = 0, T dGain = 0,
	T upperControlLimit = -1, T lowerControlLimit = 1)
	:ffGain(ffGain), pGain(pGain), iGain(iGain), dGain(dGain),
	upperControlLimit(upperControlLimit),lowerControlLimit(lowerControlLimit)
	{}
};

template<class T = float>
struct TWO_DOF_PID {
	TWO_DOF_PID(TWO_DOF_PID_PARAM<T> &param, DeltaTime *deltaTimer)
	:param(param),deltaTimer(deltaTimer){
		integral = 0;
		befState = 0;
	};
	T controller(T reference, T state);
	inline T controller(T reference, T Din, T Pin){
		return controller(reference,Din,Pin,Pin);
	}

	T controller(T reference, T Din, T Pin, T Iin);
	void setParam(TWO_DOF_PID_PARAM<T> &param){
		this->param = param;
	}
	void reset(){
		integral = 0;
		if(std::isfinite(integral) == false){
			integral = 0;
		}
	}
	T integral;
private:
	TWO_DOF_PID_PARAM<T> param;
	T befState;
	DeltaTime *deltaTimer;
};

#endif /* INC_TWO_DOF_PID_H_ */
