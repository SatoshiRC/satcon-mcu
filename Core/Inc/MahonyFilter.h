/*
 * ATTITUDE_ESTIMATION.h
 *
 *  Created on: Nov 11, 2023
 *      Author: ohya
 */

#ifndef INC_MAHONY_FILTER_H_
#define INC_MAHONY_FILTER_H_

#include "elapsedTimer/elapsedTimer.h"
#include "Quaternion/Quaternion.h"
#include "delta_time.h"

class MahonyFilter{
public:
	MahonyFilter(DeltaTime *deltaTimer = new DeltaTime(), Quaternion<float> imuFrameDiff=Quaternion<float>());

	void update();
	void updateTime();
	void updateAttitude();
	constexpr void setIsInitialized(bool arg=true){
		_isInitialized = arg;
	}
	bool isInitialized(){return _isInitialized;}

	Vector3D<float> &getGyroValue(){
		return gyroValue;
	}
	Vector3D<float> &getAccelValue(){
		return accelValue;
	}

	//These functions are used to set sensor values, gyroscope and accelerometer.
	/*
	 * brief Set accelerometer output
	 * param arg Gyroscope value in radian per second (rad/sec)
	 */
	void setGyroValue(const Vector3D<float> &arg){
		gyroValue = arg;
	}

	/*
	 * brief Set accelerometer output
	 * param arg Accelerometer value in G
	 */
	void setAccelValue(const Vector3D<float> &arg){
		accelValue = arg;
	}


	Quaternion<float> yawCancel(Quaternion<float> &attitude){
		auto heading = attitude.rotateVector({1.0,0,0});
		float xyProjectionY = std::hypot(heading[0],heading[1]);
		heading[0]/=xyProjectionY;
		heading[1]/=xyProjectionY;
		float yaw2 = 0;
		if(xyProjectionY > 0.001){
			yaw = std::atan2(heading[1],heading[0]);
			yaw2 = yaw / 2.0;
		}

		auto invYaw = Quaternion<float>(std::cos(yaw2),0,0,-std::sin(yaw2));
		return attitude * invYaw;
	}

	//getter functions
	const Quaternion<float> getAttitude(){
		return imuFrameDiff * attitude;
	}
	const float getYawRate(){
		return yawRate;
	}

private:
	DeltaTime *deltaTimer;
	Quaternion<float> imuFrameDiff;
	Quaternion<float> attitude;
	float yawRate;
	float yaw = 0;
	bool _isInitialized;
	Vector3D<float> gyroValue;
	Vector3D<float> accelValue;

	const int16_t SPIN_RATE_LIM = 300/180*std::numbers::pi;
	const float dcmKiGain = 0.000;
	const float dcmKpGain = 0.25;

	bool initialize();

	Vector3D<float> vectorOuterProduct(Vector3D<float> &arg1, Vector3D<float> &arg2);
	Vector3D<float> vectorOuterProduct(Vector3D<float> arg1, Vector3D<float> arg2);

	constexpr float square(float arg){
		return arg * arg;
	}
};

#endif /* INC_MAHONY_FILTER_H_ */
