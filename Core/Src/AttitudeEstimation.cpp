/*
 * ATTITUDE_ESTIMATION.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ohya
 */

#include "AttitudeEstimation.h"

AttitudeEstimation::AttitudeEstimation(ElapsedTimer *timer, Quaternion<float> imuFrame):timer(timer),imuFrameDiff(imuFrame) {
	// TODO Auto-generated constructor stub
	elapsedTime = timer->getTimeMS();
	deltaTime = 0;
	yawRate = 0;
	_isInitialized = false;
}

void AttitudeEstimation::updateIMU(){
	if(!_isInitialized){
		_isInitialized = initialize();
		return;
	}
	updateTime();

	Quaternion<float> deltaGyro;

	//Convert gyro coordination from sensor frame to Earth frame.
	auto gyroEarthFrame = (attitude).invers().rotateVector(gyroValue);

	//get yaw rate from gyro.
	yawRate = gyroEarthFrame[2];
	gyroEarthFrame[2] = 0;

	//get size of gyro vector
	float gyroNorm = gyroEarthFrame.getNorm();

	//To avoid 0 divide, check gyro norm has valid value.
	if(gyroNorm >= 0.001){
		//get transfer  (Earth frame to dA)
		float sinGyro = std::sin(gyroNorm/2.0);
		gyroEarthFrame.normalize();
		deltaGyro = Quaternion<float>(
			cos(gyroNorm/2.0),
			gyroEarthFrame[0]*sinGyro,
			gyroEarthFrame[1]*sinGyro,
			0
		);
	}
	deltaGyro = (deltaGyro).normalize();

	//convert quaternion coordination (Earth frame -> sensor frame)
//	deltaGyro = (attitude*deltaGyro*attitude.invers());

	//<---------calculate quaternion from accelerometer----------->//
	float accelNorm = std::abs(accelValue.getNorm() - 1.0)/1.0;

	//consider that accele value has only gravity.
	//calculate rotation axsis using outer product between accele value and gravity vector.
	Vector3D<float> rotationAxsis = vectorOuterProduct(accelValue, {0,0,1.0});
	Quaternion accelQuaternion = Quaternion<float>(1,0,0,0);
	if(rotationAxsis.getNorm() && accelNorm < 0.2){
		float cos_phi = accelValue[2] / (accelValue.getNorm());
		auto tmp = vectorOuterProduct(rotationAxsis, {0,0,1.0});
		float sin_phi = std::sqrt(1-square(cos_phi));
		float cos_phi2 = sqrt((cos_phi + 1)/2);
		float sin_phi2 = sin_phi/(2*cos_phi2);
		rotationAxsis.normalize();
		for(auto &it:rotationAxsis){
			it *= sin_phi2;
		}
//		accelQuaternion = Quaternion(cos_phi2, rotationAxsis[0], rotationAxsis[1], rotationAxsis[2]);
		accelQuaternion = Quaternion<float>(cos_phi2, rotationAxsis[0], rotationAxsis[1], 0);

	}
	auto tme = attitude.invers();
	tme[3] = 0;
	tme.normalize();
	Quaternion deltaAccelQuaternion = accelQuaternion * tme.invers();

	// float accelNorm = std::abs(receiptAccel - 1.0)/1.0;
	if(accelNorm < 0.2){
		//TODO:calculate coefficient
		auto befAttitude = attitude;
		float coefficient = 1;
		coefficient = 1.0 - square(accelNorm/0.2);
		auto delta = deltaGyro*(1-coefficient) + deltaAccelQuaternion*(coefficient);
		attitude = delta.normalize() * attitude;

		if(attitude[3]*attitude[3]>0.001){
			while(1){}
		}

		auto res = (deltaAccelQuaternion).normalize();
	}else{
		attitude = (deltaGyro).normalize() * attitude;
		if(attitude[3]*attitude[3]>0.01){
//			while(1){}
		}
	}
}

void AttitudeEstimation::updateAttitude(){
	updateIMU();

}

void AttitudeEstimation::updateTime(){
	timer->update();
	float tmp = elapsedTime;
	elapsedTime =timer->getTimeMS();
	deltaTime = (elapsedTime - tmp)/1000.0;
}

bool AttitudeEstimation::initialize(){
	return true;
}

Vector3D<float> AttitudeEstimation::vectorOuterProduct(Vector3D<float> &arg1, Vector3D<float> &arg2){
	Vector3D<float> res;
	res[0] = arg1[1]*arg2[2] - arg1[2]*arg2[1];
	res[1] = arg1[2]*arg2[0] - arg1[0]*arg2[2];
	res[2] = arg1[0]*arg2[1] - arg1[1]*arg2[0];

	return res;
}
Vector3D<float> AttitudeEstimation::vectorOuterProduct(Vector3D<float> arg1, Vector3D<float> arg2){
	Vector3D<float> res;
	res[0] = arg1[1]*arg2[2] - arg1[2]*arg2[1];
	res[1] = arg1[2]*arg2[0] - arg1[0]*arg2[2];
	res[2] = arg1[0]*arg2[1] - arg1[1]*arg2[0];

	return res;
}
