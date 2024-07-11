/*
 * MULTICOPTER.h
 *
 *  Created on: Jul 6, 2024
 *      Author: ohya
 */

#ifndef INC_MULTICOPTER_H_
#define INC_MULTICOPTER_H_

#include "Quaternion/Quaternion.h"

namespace multicopter{

struct INPUT{
	Quaternion<float> attitude;
	Quaternion<float> targetAttitude;
	float targetYawRate;
	float altitude;
	float targetAltitude;
};

struct OUTPUT{

};

struct PARAMETER{

};

struct MULTICOPTER {
	MULTICOPTER();
	void setControlParameter(PARAMETER param){
		this->_param = param;
	};
	OUTPUT controller(INPUT input);

private:
	PARAMETER _param;
};

}
#endif /* INC_MULTICOPTER_H_ */
