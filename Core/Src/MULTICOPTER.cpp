/*
 * MULTICOPTER.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#include "MULTICOPTER.h"

namespace multicopter{
std::string to_string(OUTPUT arg){
	return std::to_string((int8_t)(arg[0]*100)) + ", "
			+ std::to_string((int8_t)(arg[1]*100)) + ", "
			+ std::to_string((int8_t)(arg[2]*100)) + ", "
			+ std::to_string((int8_t)(arg[3]*100));
}


MULTICOPTER::MULTICOPTER(PARAMETER &param, DeltaTime *deltaTimer)
:_param(param),deltaTimer(deltaTimer)
{
	mainMode = MAIN_MODE::DISARM;
	
	rollController = new TWO_DOF_PID(*param.roll, deltaTimer);
	pitchController = new TWO_DOF_PID(*param.pitch, deltaTimer);
	yawRateController = new TWO_DOF_PID(*param.yawRate, deltaTimer);
	altitudeController = new TWO_DOF_PID(*param.altitude, deltaTimer);
	altitudeControlMode = param.altitudeControlMode;
	isFrameLost = false;
	elapsedTime = 0;
}

OUTPUT MULTICOPTER::controller(const INPUT &input){
	OUTPUT res={};

	float dt = deltaTimer->getDelta();

	controllerPreProcess(input);
	if(mainMode != MAIN_MODE::ARM){
		yawRateController->reset();
		rollController->reset();
		pitchController->reset();

		befInput = input;
		return res;
	}

	Vector3D<float> befAngulerRate = {};
	for(uint8_t n=0; n<0; n++){
		befAngulerRate[n] = smooth_angulerRate[n].getAverage();
	}

	smooth_angulerRate[0].push(input.rollRate);
	smooth_angulerRate[1].push(input.pitchRate);
	smooth_angulerRate[2].push(input.yawRate);

	float refRoll = input.sbusRollNorm*_param.bankAngleLimit;
	float refPitch = input.sbusPitchNorm*_param.bankAngleLimit;

	float refRollRate = sqrtController(refRoll - input.roll, dt, _param.bankAcceleLimit);
	float refPitchRate = sqrtController(refPitch - input.pitch, dt, _param.bankAcceleLimit);
	float refYawRate = -input.sbusYawRateNorm*_param.yawRateLimit;

	//Accele Limitation
//	refRollRate = min((refRollRate - angulerVel[0])/dt, _param.bankAcceleLimit);
//	refPitchRate = min((refPitchRate - angulerVel[1])/dt, _param.bankAcceleLimit);
	// refRollRate = min((refPitchRate - angulerVel[1])/dt, _param.bankAcceleLimit);

	angulerVel[0] = refRollRate;
	angulerVel[1] = refPitchRate;
	angulerVel[2] = refYawRate;

	float rollRateDiff = smooth_angulerRate[0].getAverage() - befAngulerRate[0];
	float pitchRateDiff = smooth_angulerRate[1].getAverage() - befAngulerRate[1];
	float yawRateDiff = smooth_angulerRate[2].getAverage() - befAngulerRate[2];

	std::array<float, 4> u;
	u.at(3) = yawRateController->controller(refYawRate,yawRateDiff,refYawRate - smooth_angulerRate[2].getAverage());
	u.at(1) = rollController->controller(refRollRate,rollRateDiff,refRollRate - smooth_angulerRate[0].getAverage());
	u.at(2) = pitchController->controller(refPitchRate,pitchRateDiff,refPitchRate - smooth_angulerRate[1].getAverage());

//	u.at(1) = rollController->controller(refRollRate, input.rollRate);
//	u.at(2) = pitchController->controller(refPitchRate, input.pitchRate);
//	u.at(3) = yawRateController->controller(refYawRate, input.yawRate);

	switch(altitudeControlMode){
		case ALTITUDE_CONTROL_MODE::ALTITUDE_FEEDBACK:
			u.at(0) = altitudeController->controller(input.sbusAltitudeNorm,input.altitude);
			break;
		case ALTITUDE_CONTROL_MODE::THROTTLE:
			u.at(0) = (input.sbusAltitudeNorm + 1.0) * 0.5;
			break;
		case ALTITUDE_CONTROL_MODE::RELATIVE_THROTTLE:
			u.at(0) = integrateThrottle(input.sbusAltitudeNorm, dt);
	}

//	linarization(u);

//	for(auto &it:u){
//		it = std::copysign(std::sqrt(std::abs(it)), it);
//	}

	controlValue = u;

	res[0] = u[0] - u[1] + u[2] + u[3];
	res[1] = u[0] - u[1] - u[2] - u[3];
	res[2] = u[0] + u[1] + u[2] - u[3];
	res[3] = u[0] + u[1] - u[2] + u[3];

	if(std::isfinite(res[0])==false){
		res[0] = 0;
	}

	for(auto &it:res){
		constraintFloat(it,0,1);
		it = std::sqrt(it);
	}

//	for(auto &it:res){
//		if(it<0){
//			it = 0;
//		}else if(it>1.0){
//			it = 1.0;
//		}else{
//			it = std::pow(it,2);
//		}
//	}

	befInput = input;
	return res;

}

void MULTICOPTER::linarization(std::array<float, 4> &u){

}

void MULTICOPTER::controllerPreProcess(const INPUT &input){
	std::function<bool(const INPUT)> isArming =[](const INPUT &input){
		if(input.sbusAltitudeNorm < -0.99){
			if(input.sbusYawRateNorm > 0.99){
				return true;
			}
		}
		return false;
	};
	std::function<bool(const INPUT)>  isDisArming = [](const INPUT &input){
		if(input.sbusAltitudeNorm < -0.99){
			if(input.sbusYawRateNorm < -0.99){
				return true;
			}
		}
		return false;
	};

	if(mainMode == MAIN_MODE::DISARM && isArming(input)){
		mainMode = MAIN_MODE::ARMING;
		armingMotionStart = deltaTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::ARMING){
		if(isArming(input)){
			if(armingMotionStart != 0 && deltaTimer->getTimeMS() - armingMotionStart >= armingDurationTH){
				//TODO:pre arm check
				mainMode = MAIN_MODE::ARM;
			}
		}else{
			mainMode = MAIN_MODE::DISARM;
			armingMotionStart = 0;
		}
	}else if(mainMode == MAIN_MODE::ARM && isDisArming(input)){
		mainMode = MAIN_MODE::DISARMING;
		armingMotionStart = deltaTimer->getTimeMS();
	}else if(mainMode == MAIN_MODE::DISARMING){
		if(isDisArming(input)){
			if(armingMotionStart != 0 &&  deltaTimer->getTimeMS() - armingMotionStart >= armingDurationTH){
				mainMode = MAIN_MODE::DISARM;
			}
		}else{
			mainMode = MAIN_MODE::ARM;
			armingMotionStart = 0;
		}
	}
}

float MULTICOPTER::integrateThrottle(float throttleNorm, float dt){
	float deadZone = 0.01;
	float element = 0;
	if(std::abs(throttleNorm) < deadZone){
		element = 0;
	}else{
		if(throttleNorm > 0){
			element = throttleNorm - deadZone;
		}else{
			element = throttleNorm + deadZone;
		}
		constraintFloat(element,-1,1);

		throttle_integral += element * dt * 2.0;
	}
	constraintFloat(throttle_integral,0,1);
	return throttle_integral;
}

float MULTICOPTER::sqrtController(float error, float deltaT, float secondOrderLim){
	float res = 0;
	
	if(error>0){
		res = sqrt(2.0*secondOrderLim*error-std::pow(secondOrderLim*deltaT,2));
	}else if(error < 0){
		res = -sqrt(2.0*secondOrderLim*(-error)-std::pow(secondOrderLim*deltaT,2));
	}else{
		res = 0;
	}

	constraintFloat(res, -std::abs(error)/deltaT, std::abs(error)/deltaT);

	if(std::isfinite(res) == false){
		res = 0;
	}
	return res;
}

std::string MULTICOPTER::getCotrolValue(){
	return std::to_string((int16_t)(controlValue[0]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[1]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[2]*100)) + ", "
			+ std::to_string((int16_t)(controlValue[3]*100));
}

std::string MULTICOPTER::getRefValue(){
	return std::to_string((int16_t)(angulerVel[0]*180 / std::numbers::pi)) + ", "
				+ std::to_string((int16_t)(angulerVel[1]*180 / std::numbers::pi)) + ", "
				+ std::to_string((int16_t)(angulerVel[2]*180 / std::numbers::pi));
}

std::string MULTICOPTER::getSmoothValue(){
	return std::to_string((int16_t)(smooth_angulerRate[0].getAverage()*180 / std::numbers::pi)) + ", "
				+ std::to_string((int16_t)(smooth_angulerRate[1].getAverage()*180 / std::numbers::pi)) + ", "
				+ std::to_string((int16_t)(smooth_angulerRate[2].getAverage()*180 / std::numbers::pi));
}

}
