/*
 * MULTICOPTER.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#include "/home/ohya/STM32CubeIDE/workspace_1.12.1/PixBee_P4_MultiCopter/Core/Inc/MULTICOPTER.h"

namespace multicopter{

MULTICOPTER::MULTICOPTER(PARAMETER &param, ElapsedTimer *elapsedTimer)
:_param(param),elapsedTimer(elapsedTimer)
{
	mainMode = MAIN_MODE::DISARM;
	
	rollController = new TWO_DOF_PID(param.roll);
	pitchController = new TWO_DOF_PID(param.pitch);
	yawRateController = new TWO_DOF_PID(param.yawRate);
	altitudeControlMode = param.altitudeControlMode;
}

OUTPUT MULTICOPTER::controller(const INPUT &input){
	OUTPUT res;

	controllerPreProcess(input);
	if(mainMode != MAIN_MODE::ARM){
		return res;
	}


	float refRoll = input.sbusRollNorm*_param.bankAngleLimit;
	float refPitch = input.sbusPitchNorm*_param.bankAngleLimit;
	float refYawRate = input.sbusRollNorm*_param.yawRateLimit;

	std::array<float, 4> u;
	u.at(3) = yawRateController.controller(refYawRate,input.yawRate);
	u.at(1) = rollController.controller(refRoll,input.roll);
	u.at(2) = pitchController.controller(refPitch,input.pitch);

	switch(ALTITUDE_CONTROL_MODE){
		case ALTITUDE_CONTROL_MODE::ALTITUDE_FEEDBACK:
			u.at(0) = altitudeController.controller(input.refAltitude,input.altitude);
			break;
		case ALTITUDE_CONTROL_MODE::THROTTLE:
			u.at(0) = input.thurottle;
			break;
	}

	res.motor0 = u[0] + u[2] + u[3];
	res.motor1 = u[0] - u[1] - u[3];
	res.motor2 = u[0] - u[2] + u[3];
	res.motor3 = u[0] + u[2] - u[3];

}

void MULTICOPTER::controllerPreProcess(const INPUT &input){
	bool isArming(const INPUT &input){
		if(input.thurottle < -0.99){
			if(input.yawRate > 0.99){
				return true;
			}
		}
		return false;
	}
	bool isDisArming(const INPUT &input){
		if(input.thurottle < -0.99){
			if(input.yawRate < -0.99){
				return true;
			}
		}
		return false;
	}

	if(mainMode == MAIN_MODE::DISARM && isArming()){
		mainMode = MAIN_MODE::ARMING;
		armingMotionStart = elapsedTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::ARMING){
		if(isArming()){
			if(armingMotionStart != 0 && armingMotionStart - elapsedTimer->getTimeMS() >= armingDurationTH){
				mainMode = MAIN_MODE::ARM;
			}
		}else{
			mainMode = MAIN_MODE::DISARM;
			armingMotionStart = 0;
		}
	}else if(mainMode == MAIN_MODE::ARM && isDisArming()){
		mainMode = MAIN_MODE::DISARMING;
		armingMotionStart = elapsedTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::DISARMING){
		if(isDisArming()){
			if(armingMotionStart != 0 && armingMotionStart - elapsedTimer->getTimeMS() >= armingDurationTH){
				mainMode = MAIN_MODE::DISARM;
			}
		}else{
			mainMode = MAIN_MODE::ARM;
			armingMotionStart = 0;
		}
	}
	
}


}
