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
#include "MovingAverage/movingAverage.h"
#include "TWO_DOF_PID.h"
#include "functional"
#include "string"
#include "delta_time.h"

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
typedef std::array<float, 8> OUTPUT;
std::string to_string(OUTPUT arg);

enum class ALTITUDE_CONTROL_MODE{
	THROTTLE,
	RELATIVE_THROTTLE,
	ALTITUDE_FEEDBACK,
};

enum class MAIN_MODE{
	DISARM,
	ARMING,
	ARM,
	DISARMING,
};

enum class FRAME_TYPE{
	OCTA,
	QUAD_TOP,
	QUAD_BOTTOM,
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
	FRAME_TYPE frameType;

	PARAMETER(TWO_DOF_PID_PARAM<float> *roll,
	TWO_DOF_PID_PARAM<float> *pitch,
	TWO_DOF_PID_PARAM<float> *yawRate,
	TWO_DOF_PID_PARAM<float> *altitude,
	ALTITUDE_CONTROL_MODE altitudeControlMode = ALTITUDE_CONTROL_MODE::THROTTLE,
	float bankAngleLimit = 0.1745,
	float bankAcceleLimit = 15,
	float yawRateLimit = 0.1745,
	FRAME_TYPE frameType = FRAME_TYPE::QUAD_TOP)
	:roll(roll), pitch(pitch), yawRate(yawRate), altitude(altitude),
	bankAngleLimit(bankAngleLimit), yawRateLimit(yawRateLimit), bankAcceleLimit(bankAcceleLimit),
	altitudeControlMode(altitudeControlMode),frameType(frameType)
	{}
};

struct MULTICOPTER {
	MULTICOPTER(PARAMETER &param, DeltaTime *deltaTimer);
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

	std::string getRefValue();

	std::string getSmoothValue();

	Vector3D<MovingAverage<float, 10>> smooth_angulerRate;
private:
	void linarization(std::array<float, 4> &u);

	PARAMETER _param;
	TWO_DOF_PID<float> *rollController;
	TWO_DOF_PID<float> *pitchController;
	TWO_DOF_PID<float> *yawRateController;
	TWO_DOF_PID<float> *altitudeController;
	ALTITUDE_CONTROL_MODE altitudeControlMode;
	MAIN_MODE mainMode;
	DeltaTime *deltaTimer;
	uint64_t elapsedTime;
	Vector3D<float> angulerVel;
	Vector3D<float> refRate;

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

	/*
	 * @brief : integrate throttle in the case of RELATIVE_THROTTLE mode
	 * @params : normalized throttle value as -1 to 1
	 * @params : step time in seconds	
	 */
	float integrateThrottle(float throttleNorm, float dt);
	float throttle_integral = 0;
};


}
#endif /* INC_MULTICOPTER_H_ */
