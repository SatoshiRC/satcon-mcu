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
	float altitude;
	float thurottle;
	
	float refRoll;
	float refPitch;
	float refYawRate;
	float refAltitude;
};

//give the motor output in persent(%)
struct OUTPUT{
	float motor0;
	float motor1;
	float motor2;
	float motor3;
};

struct PARAMETER{
	TWO_DOF_PID_PARAM *roll;
	TWO_DOF_PID_PARAM *pitch;
	TWO_DOF_PID_PARAM *yawRate;
	TWO_DOF_PID_PARAM *altitude;
	float bankAngleLimit;	//give roll and pitch limit angle in rad
	float yawRateLimit;		//give yaw rate limit in rad per sec

	PARAMETER(TWO_DOF_PID_PARAM *roll,
	TWO_DOF_PID_PARAM *pitch,
	TWO_DOF_PID_PARAM *yawRate,
	TWO_DOF_PID_PARAM *altitude,
	float bankAngleLimit = 0.1745,
	float yawRateLimit = 0.1745)
	:roll(roll), pitch(pitch), yawRate(yawRate), altitude(altitude),
	bankAngleLimit(bankAngleLimit), yawRateLimit(yawRateLimit)
	{}
};

enum class ALTITUDE_CONTROL_MODE{
	THROTTLE,
	ALTITUDE_FEEDBACK,
}

struct MULTICOPTER {
	MULTICOPTER(PARAMETER param = PARAMETER(), ALTITUDE_CONTROL_MODE altitudeControlMode = ALTITUDE_CONTROL_MODE::THROTTLE);
	void setControlParameter(PARAMETER param){
		this->_param = param;
	};
	OUTPUT controller(INPUT input);
	void setAltitudeControlMode(ALTITUDE_CONTROL_MODE mode){
		altitudeControlMode = mode;
	}
	void rcPreProcess(std::array<uint16_t, 18> &rc, INPUT &input);

private:
	PARAMETER _param;
	TWO_DOF_PID rollController;
	TWO_DOF_PID pitchController;
	TWO_DOF_PID yawRateController;
	TWO_DOF_PID altitudeController;
	ALTITUDE_CONTROL_MODE altitudeControlMode;
};

}
#endif /* INC_MULTICOPTER_H_ */
