/*
 * MULTICOPTER.cpp
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#include "/home/ohya/STM32CubeIDE/workspace_1.12.1/PixBee_P4_MultiCopter/Core/Inc/MULTICOPTER.h"

namespace multicopter{

MULTICOPTER::MULTICOPTER(PARAMETER param, ALTITUDE_CONTROL_MODE altitudeControlMode)
:_param(param),
altitudeControlMode(altitudeControlMode) {
	// TODO Auto-generated constructor stub
	rollController = TWO_DOF_PID(param->roll);
	pitchController = TWO_DOF_PID(param->pitch);
	yawRateController = TWO_DOF_PID(param->yawRate);
}

OUTPUT MULTICOPTER::controller(INPUT input){
	OUTPUT res;

	std::array<float, 4> u;
	u.at(3) = yawRateController.controller(input.refYawRate,input.yawRate);
	u.at(1) = rollController.controller(input.refRoll,input.roll);
	u.at(2) = pitchController.controller(input.refPitch,input.pitch);

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

void MULTICOPTER::rcPreProcess(std::array<uint16_t, 18> &rc, INPUT &input){

}


}
