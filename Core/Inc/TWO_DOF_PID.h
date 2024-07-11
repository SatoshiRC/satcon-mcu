/*
 * TWO_DOF_PID.h
 *
 *  Created on: Jul 12, 2024
 *      Author: ohya
 */

#ifndef INC_TWO_DOF_PID_H_
#define INC_TWO_DOF_PID_H_

template<class T = float>
struct TWO_DOF_PID_PARAM{
	T ffGain = 0;
	T pGain = 0;
	T iGain = 0;
	T dGain = 0;
	T upperControlLimit = 0;
	T lowerControlLimit = 0;
};

template<class T = float>
struct TWO_DOF_PID {
	TWO_DOF_PID(TWO_DOF_PID_PARAM<T> &param):param(param){};
	T controller(T reference, T state);
	void setParam(TWO_DOF_PID_PARAM<T> &param){
		this->param = param;
	}
private:
	TWO_DOF_PID_PARAM<T> param;
	T integral;
	T befState;
};

#endif /* INC_TWO_DOF_PID_H_ */
