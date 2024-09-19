/*
 * ATTITUDE_ESTIMATION.cpp
 *
 *  Created on: Nov 11, 2023
 *      Author: ohya
 */

#include "MahonyFilter.h"

MahonyFilter::MahonyFilter(DeltaTime *deltaTimer, Quaternion<float> imuFrame):deltaTimer(deltaTimer),imuFrameDiff(imuFrame) {
	// TODO Auto-generated constructor stub
	_isInitialized = false;
}

void MahonyFilter::update(){
	_isInitialized = true;

	float dt = deltaTimer->getDelta();
	static Vector3D<float> integralFB={};    // integral error terms scaled by Ki

	// Calculate general spin rate (rad/s)
	const float spin_rate = gyroValue.norm();

	Vector3D<float> error={};
	Vector3D<float> omega={};

	// Use measured acceleration vector
	float recipAccNorm = accelValue.norm();
	accelValue.normalize();
	if (recipAccNorm > 0.9f && recipAccNorm < 1.1f) {
		// Error is sum of cross product between estimated direction and measured direction of gravity
		Vector3D<float> refGravity = attitude.invers().rotateVector({0,0,1});

		error = accelValue.outerProduct(refGravity);
	}

	// Compute and apply integral feedback if enabled
	if (dcmKiGain > 0.0f) {
		// Stop integrating if spinning beyond the certain limit
		if (spin_rate < (SPIN_RATE_LIM)) {
			integralFB = integralFB - (error*dcmKiGain*dt);// integral error scaled by Ki
		}
	} else {
		integralFB = {0,0,0};
	}

	// Apply proportional and integral feedback
	omega = gyroValue + error*dcmKpGain + integralFB;

	// Integrate rate of change of quaternion
	Quaternion qDot = attitude*Quaternion(omega);
	attitude = attitude + qDot*0.5*dt;

	// Normalise quaternion
	attitude.normalize();

}

void MahonyFilter::updateAttitude(){

}

bool MahonyFilter::initialize(){
	return true;
}

Vector3D<float> MahonyFilter::vectorOuterProduct(Vector3D<float> &arg1, Vector3D<float> &arg2){
	Vector3D<float> res;
	res[0] = arg1[1]*arg2[2] - arg1[2]*arg2[1];
	res[1] = arg1[2]*arg2[0] - arg1[0]*arg2[2];
	res[2] = arg1[0]*arg2[1] - arg1[1]*arg2[0];

	return res;
}
Vector3D<float> MahonyFilter::vectorOuterProduct(Vector3D<float> arg1, Vector3D<float> arg2){
	Vector3D<float> res;
	res[0] = arg1[1]*arg2[2] - arg1[2]*arg2[1];
	res[1] = arg1[2]*arg2[0] - arg1[0]*arg2[2];
	res[2] = arg1[0]*arg2[1] - arg1[1]*arg2[0];

	return res;
}
