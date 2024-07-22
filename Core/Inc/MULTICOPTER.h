/*
 * MULTICOPTER.h
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#ifndef INC_MULTICOPTER_H_
#define INC_MULTICOPTER_H_

#include "Quaternion/Quaternion.h"
#include "TWO_DOF_PID.h"

namespace multicopter{

struct INPUT{
	float roll;
	float pitch;
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
struct OUTPUT{
	float motor0;
	float motor1;
	float motor2;
	float motor3;

	OUTPUT(){
		motor0 = 0;
		motor1 = 0;
		motor2 = 0;
		motor3 = 0;
	}
};

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
	TWO_DOF_PID_PARAM *roll;
	TWO_DOF_PID_PARAM *pitch;
	TWO_DOF_PID_PARAM *yawRate;
	TWO_DOF_PID_PARAM *altitude;
	float bankAngleLimit;	//give roll and pitch limit angle in rad
	float yawRateLimit;		//give yaw rate limit in rad per sec

	ALTITUDE_CONTROL_MODE altitudeControlMode;

	PARAMETER(TWO_DOF_PID_PARAM *roll,
	TWO_DOF_PID_PARAM *pitch,
	TWO_DOF_PID_PARAM *yawRate,
	TWO_DOF_PID_PARAM *altitude,
	ALTITUDE_CONTROL_MODE altitudeControlMode = ALTITUDE_CONTROL_MODE::THROTTLE,
	float bankAngleLimit = 0.1745,
	float yawRateLimit = 0.1745)
	:roll(roll), pitch(pitch), yawRate(yawRate), altitude(altitude),
	bankAngleLimit(bankAngleLimit), yawRateLimit(yawRateLimit), 
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
	void rcFrameLost(){};
	void rcFailSafe(){};
	void getMainMode(){
		return mainMode;
	}
private:
	PARAMETER _param;
	TWO_DOF_PID *rollController;
	TWO_DOF_PID *pitchController;
	TWO_DOF_PID *yawRateController;
	TWO_DOF_PID altitudeController;
	ALTITUDE_CONTROL_MODE altitudeControlMode;
	MAIN_MODE mainMode;
	ElapsedTimer *elapsedTimer;

	void controllerPreProcess(const INPUT &input);
	float armingMotionStart = 0;
	float armingDurationTH = 1000; //ms
};

}
#endif /* INC_MULTICOPTER_H_ */
