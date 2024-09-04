/*
 * MULTICOPTER.h
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#ifndef INC_MULTICOPTER_H_
#define INC_MULTICOPTER_H_

#include "Quaternion/Quaternion.h"
#include "Quaternion/Vector3D/Vector3D.h"
#include "TWO_DOF_PID.h"
#include "functional"
#include "string"

namespace multicopter{

struct INPUT{
	float roll;
	float pitch;

	float rollRate;
	float pitchRate;
	float yawRate;
	float thurottle;

	float altitude;
	
	//give the input of normalized sbus data
	float sbusRollNorm;
	float sbusPitchNorm;
	float sbusYawRateNorm;
	float sbusAltitudeNorm;

	bool updateFlag = 0;
};

//give the motor output in persent(%)
typedef std::array<float, 4> OUTPUT;
std::string to_string(OUTPUT arg);

enum class ALTITUDE_CONTROL_MODE{
	THROTTLE,
	ALTITUDE_FEEDBACK,
};

enum class MAIN_MODE{
	DISARM,
	ARMING,
	ARM,
	DISARMING,
};

struct PARAMETER{
	TWO_DOF_PID_PARAM<float> *roll;
	TWO_DOF_PID_PARAM<float> *pitch;
	TWO_DOF_PID_PARAM<float> *yawRate;
	TWO_DOF_PID_PARAM<float> *altitude;
	float bankAngleLimit;	//give roll and pitch limit angle in rad
	float yawRateLimit;		//give yaw rate limit in rad per sec
	float bankAcceleLimit;

	ALTITUDE_CONTROL_MODE altitudeControlMode;

	PARAMETER(TWO_DOF_PID_PARAM<float> *roll,
	TWO_DOF_PID_PARAM<float> *pitch,
	TWO_DOF_PID_PARAM<float> *yawRate,
	TWO_DOF_PID_PARAM<float> *altitude,
	ALTITUDE_CONTROL_MODE altitudeControlMode = ALTITUDE_CONTROL_MODE::THROTTLE,
	float bankAngleLimit = 0.1745,
	float bankAcceleLimit = 15,
	float yawRateLimit = 0.1745)
	:roll(roll), pitch(pitch), yawRate(yawRate), altitude(altitude),
	bankAngleLimit(bankAngleLimit),  bankAcceleLimit(bankAcceleLimit), yawRateLimit(yawRateLimit),
	altitudeControlMode(altitudeControlMode)
	{}
};

struct MULTICOPTER {
	MULTICOPTER(PARAMETER &param, ElapsedTimer *elapsedTimer);
	void setControlParameter(PARAMETER param){
		this->_param = param;
	};
	OUTPUT controller(const INPUT &input);
	void setAltitudeControlMode(ALTITUDE_CONTROL_MODE mode){
		altitudeControlMode = mode;
	}
	void setRcFrameLost(bool isFrameLost = true){
		this->isFrameLost = isFrameLost;
	};
	void rcFailSafe(){
		mainMode = MAIN_MODE::DISARM;
	};
	MAIN_MODE getMainMode(){
		return mainMode;
	}

	std::string getCotrolValue();
private:
	void linarization(std::array<float, 4> &u);

	PARAMETER _param;
	TWO_DOF_PID<float> *rollController;
	TWO_DOF_PID<float> *pitchController;
	TWO_DOF_PID<float> *yawRateController;
	TWO_DOF_PID<float> *altitudeController;
	ALTITUDE_CONTROL_MODE altitudeControlMode;
	MAIN_MODE mainMode;
	ElapsedTimer *elapsedTimer;
	float elapsedTime;
	Vector3D<float> angulerVel;

	bool isFrameLost;

	void controllerPreProcess(const INPUT &input);
	float armingMotionStart = 0;
	float armingDurationTH = 1000; //ms

	INPUT befInput = INPUT();

	std::array<float, 4> controlValue;

	static float max(float one, float two){
		if(one<two){
			return two;
		}
		return one;
	}

	static float min(float one, float two){
		if(one<two){
			return one;
		}
		return two;
	}

	float sqrtController(float error, float deltaT, float secondOrderLim);
	inline void constraintFloat(float &in, float min, float max){
		in = in<min?min:in;
		in = in>max?max:in;
	}
};

}
#endif /* INC_MULTICOPTER_H_ */
