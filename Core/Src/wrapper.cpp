/*
 * wrapper.cpp
 *
 *  Created on: Oct 23, 2023
 *      Author: ohya
 */

#include "wrapper.hpp"

#include "common.h"

#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "gpio.h"
#include <string>
#include <array>
#include <bitset>
#include <exception>
#include <cmath>
#include <functional>

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
static void message(std::string str, uint8_t level = 3);

std::array<float, 3> gyroValue;
std::array<float, 3> AccelValue;

void init(){
	SET_MASK_ICM20948_INTERRUPT();

	HAL_TIM_PWM_Start(RED_LED);
	HAL_TIM_PWM_Start(YELLOW_LED);
	HAL_TIM_PWM_Start(BLUE_LED);

	__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 100);
	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 100);
	__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 100);
	HAL_Delay(1000);
	__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 0);


//	esc.enable();
//
//	HAL_Delay(5000);
//	for(uint8_t n=0; n<8; n++){
//		std::array<float, 8> sp={};
//		sp[n] = 0.1;
//		esc.setSpeed(sp);
//		HAL_Delay(3000);
//		esc.setSpeed(0);
//	}

	//start to receive sbus.
	HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());

	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);
	__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);


	if(elapsedTimer->selfTest() == false){
		message("ERROR : elapsed timer freaquency is not correct",0);
	}else{
		message("elapsed timer is working",2);
		__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 200);
	}
	elapsedTimer->start();

	/*
	 * communication check with icm20948
	 * initialize icm20948
	 */
	try{
		icm20948User.confirmConnection();
		message("ICM20948 is detected",2);
		__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 300);
	}catch(std::runtime_error &e){
		message("Error : Icm20948 is not detected",0);
	}
	icm20948User.init();
	_icm20948Callback = icm20948CallbackCalibration;
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	while(icm20948User.isCalibrated() == false){
		HAL_Delay(100);
	}
	message("icm20948 calibration is finished",3);
	_icm20948Callback = icm20948Callback;

	while(attitudeEstimate.isInitialized() == false){
		HAL_Delay(50);
	}
	message("attitude estimation method is initialized");
	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 400);
	HAL_Delay(100);


	esc.enable();
	esc.arm();

	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 500);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	isInitializing = false;
	message("Initialization is complete",1);
}

void loop(){
	HAL_Delay(100);
	if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::ARM){
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	}else if(hmulticopter->getMainMode() == multicopter::MAIN_MODE::DISARM){
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 0);
	}else{
		__HAL_TIM_SET_COMPARE(ledTim, BLUE_LED_CHANNEL, 250);
	}
}

void icm20948CallbackCalibration(){
	Vector3D<float> accel;
	Vector3D<float> gyro;
	icm20948->readIMU();
	icm20948->getIMU(accel, gyro);
	icm20948User.calibration(gyro);
}

void icm20948Callback(){
	Vector3D<float> accel;
	Vector3D<float> gyro;

	deltaTimer->update_dt();

	icm20948User.getIMU(accel, gyro);

	attitudeEstimate.setAccelValue(accel);
	attitudeEstimate.setGyroValue(gyro);
	attitudeEstimate.update();

	if(isInitializing){
		return;
	}

//	multicopterInput.rollRate = rollFilter.filter(gyro[1]);
//	multicopterInput.rollRate = 0.2*multicopterInput.rollRate - 0.8*gyro[0];
//	multicopterInput.pitchRate = 0.2*multicopterInput.pitchRate - 0.8*gyro[1];;
//	multicopterInput.yawRate = 0.2*multicopterInput.yawRate - 0.8*gyro[2];

//	multicopterInput.rollRate = rollFilter.filter(gyro[1]);
	multicopterInput.rollRate = -gyro[0];
	multicopterInput.pitchRate = gyro[1];;
	multicopterInput.yawRate = gyro[2];

	auto attitude = attitudeEstimate.getAttitude();

	auto heading = attitude.rotateVector({1.0,0,0});
	float xyProjectionY = std::hypot(heading[0],heading[1]);
	heading[0]/=xyProjectionY;
	heading[1]/=xyProjectionY;
	float yaw = 0;
	float yaw2 = 0;
	if(xyProjectionY > 0.001){
		yaw = std::atan2(heading[1],heading[0]);
		yaw2 = yaw / 2.0;
	}
	auto invYaw = Quaternion<float>(std::cos(yaw2),0,0,-std::sin(yaw2));
	attitude = attitude * invYaw;

	auto z_machienFrame = attitude.rotateVector({0,0,1.0});
	float xyProjectionZ = std::hypot(z_machienFrame[0],z_machienFrame[1]);
	float _roll = 0;
	float _pitch = 0;
	if(xyProjectionZ > 0.001){
		_roll = std::asin(z_machienFrame[0]);
		_pitch = -std::asin(z_machienFrame[1]);
	}
	auto tmpVec = invYaw.rotateVector({_pitch,_roll,0});
	float roll = -tmpVec[0];
	float pitch = tmpVec[1];
//	float yawRate = gyro[2];

	multicopterInput.roll = roll;
	multicopterInput.pitch = pitch;
//	multicopterInput.yawRate = yawRate;
	auto res = hmulticopter->controller(multicopterInput);
	esc.setSpeed(res);

//	message(gyro.string() +", " +accel.string2());
//	message(std::to_string(uint16_t(deltaTimer->getDelta()*1000*1000)));
//	message(std::to_string(int16_t(roll*180/std::numbers::pi))+", "+std::to_string(int16_t(pitch*180/std::numbers::pi))+", "+std::to_string(int16_t(yaw*180/std::numbers::pi))+", "+hmulticopter->getSmoothValue());
//	message(std::to_string(int16_t(yaw*180/std::numbers::pi))+", "+ std::to_string(int16_t(hmulticopter->smooth_angulerRate[2].getAverage()*180/std::numbers::pi)) + ", " + std::to_string(int16_t(gyro[2]*180/std::numbers::pi)));
//	message(std::to_string(int16_t(roll*180/std::numbers::pi))+", "+ std::to_string(int16_t(hmulticopter->smooth_angulerRate[0].getAverage()*180/std::numbers::pi)) + ", " + std::to_string(int16_t(multicopterInput.rollRate*180/std::numbers::pi)) + ", " + std::to_string(int16_t(gyro[0]*180/std::numbers::pi)));
//	message(multicopter::to_string(res)+", "+hmulticopter->getCotrolValue(), 3);
//	message(hmulticopter->getRefValue()+", "+hmulticopter->getSmoothValue()+", "+hmulticopter->getCotrolValue()+", "+multicopter::to_string(res),3);
//	message(hmulticopter->getCotrolValue(), 3);
//	message(std::to_string(int16_t(	multicopterInput.rollRate*180/std::numbers::pi))+", "+std::to_string(int16_t(multicopterInput.pitchRate*180/std::numbers::pi)));


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ICM20948_IT_Pin){
		if(HAL_GPIO_ReadPin(ICM20948_IT_GPIO_Port, ICM20948_IT_Pin) == GPIO_PIN_SET){
			_icm20948Callback();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartSbus){
		htim14.Instance->CNT = 0;
		hsbus.onReceive(multicopterInput);
		if(hsbus.getData().failsafe){
			hmulticopter->rcFailSafe();
		}else if(hsbus.getData().framelost){
			hmulticopter->setRcFrameLost();
			esc.setSpeed(0);
		}else{
			hmulticopter->setRcFrameLost(false);
		}
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());

		if(hsbus.getData().failsafe || hsbus.getData().framelost){
			return;
		}

//		esc.setSpeed(hmulticopter->controller(multicopterInput));
		std::string str;

//		for(uint8_t n=0; n<10; n++){
//			str += std::to_string(hsbus.getData()[n])+", ";
//		}
//		str= std::to_string(int8_t(multicopterInput.sbusRollNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusPitchNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusYawRateNorm*100))+", ";
//		str += std::to_string(int8_t(multicopterInput.sbusAltitudeNorm*100))+", ";
//		message(str, 3);
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 500);
	}else if(huart == huartDebug){

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == huartDebug){
		if(huart->RxEventType == HAL_UART_RXEVENT_TC){

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartDebug){

	}
}

static void message(std::string str, uint8_t level){
	if(level <= messageLevel){
		str += "\n";
		if(huartDebug->gState == HAL_UART_STATE_READY){
			static std::string messageBuffer;
			messageBuffer = std::string(str);
			HAL_UART_Transmit_DMA(huartDebug, (uint8_t *)messageBuffer.c_str(), messageBuffer.length());
		}
	}
	return;
}

void tim14Callback(){
	auto htim = &htim14;
	uint16_t sr = htim->Instance->SR;
	if((sr & (TIM_IT_CC1)) == (TIM_IT_CC1)){
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 100);
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_CC1);
		HAL_UART_AbortReceive(huartSbus);
		HAL_DMA_Abort(huartSbus->hdmarx);
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		hmulticopter->setRcFrameLost();
	}else if((sr & TIM_IT_UPDATE) == (TIM_IT_UPDATE)){
		__HAL_TIM_SET_COMPARE(ledTim, RED_LED_CHANNEL, 300);
		HAL_DMA_Abort(huartSbus->hdmarx);
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		__HAL_TIM_CLEAR_FLAG(htim,TIM_IT_UPDATE);
		hmulticopter->rcFailSafe();
//		esc.setSpeed(0);
	}
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart3){
//		huart->gState = HAL_UART_STATE_READY;
//	}
//}
