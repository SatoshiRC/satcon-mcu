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
#include "adc.h"
#include <string>
#include <array>
#include <bitset>
#include <exception>
#include <cmath>
#include <functional>

#define USE_XBEE 1

#if USE_XBEE
UART_HandleTypeDef *huartDebug = &huart2;
#else
UART_HandleTypeDef *huartDebug = &huart4;
#endif

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
static void message(std::string str, uint8_t level = 3);
uint8_t transmitIMU(Vector3D<int16_t> &accel, Vector3D<int16_t> &gyro);

std::array<uint8_t, 20> rxBuffer={};
float start = 0;

std::array<std::array<Vector3D<int16_t>, 2>, 21000> imuBuffer = {};
uint32_t imuDataCounter = 0;
bool isMode2 = false;

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
	_icm20948Callback = icm20948Callback;
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 100);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	isInitializing = false;
	message("Initialization is complete",1);

//	HAL_UART_Receive_IT(huartDebug, rxBuffer.data(), rxBuffer.size());
	HAL_UARTEx_ReceiveToIdle_IT(huartDebug, rxBuffer.data(), rxBuffer.size());
}

void loop(){
	if(isMode2){
		if(elapsedTimer->getTimeMS() - start >= 18*1000){
			SET_MASK_ICM20948_INTERRUPT();
			__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 400);
			// TODO: transmit sensor data;

			for(uint32_t n=0; n<imuDataCounter; n++){
				while(huartDebug->gState != HAL_UART_STATE_READY);
				HAL_Delay(1);
				transmitIMU(imuBuffer[n][0], imuBuffer[n][1]);
			}

			HAL_Delay(100);
			message("end");

			_icm20948Callback = icm20948Callback;
			CLEAR_MASK_ICM20948_INTERRUPT();
			isMode2 = false;
			HAL_UARTEx_ReceiveToIdle_IT(huartDebug, rxBuffer.data(), rxBuffer.size());
			__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 100);
		}
	}

}

void icm20948Callback(){
	Vector3D<int16_t> accel;
	Vector3D<int16_t> gyro;

	static uint8_t n = 0;

	deltaTimer->update_dt();
	float dt = deltaTimer->getDelta();

	icm20948User.getIMU(accel, gyro);
	if(n++ % 3 == 0){
	auto res =transmitIMU(accel, gyro);
		if(res){
			__HAL_TIM_SET_COMPARE(ledTim,RED_LED_CHANNEL,250);
			__HAL_TIM_SET_COMPARE(ledTim,BLUE_LED_CHANNEL,0);
		}else{
			__HAL_TIM_SET_COMPARE(ledTim,RED_LED_CHANNEL,0);
			__HAL_TIM_SET_COMPARE(ledTim,BLUE_LED_CHANNEL,500);
		}
	}
//	std::string str = std::to_string((uint16_t)(dt*1000000));
//	message(str);
}

void icm20948Callback2(){
	__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 500);
	Vector3D<int16_t> accel;
	Vector3D<int16_t> gyro;

	if(imuDataCounter >= imuBuffer.size()){
		return;
	}
	icm20948User.getIMU(accel, gyro);
	imuBuffer[imuDataCounter][0] = accel;
	imuBuffer[imuDataCounter][1] = gyro;
	imuDataCounter++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == ICM20948_IT_Pin){
		if(HAL_GPIO_ReadPin(ICM20948_IT_GPIO_Port, ICM20948_IT_Pin) == GPIO_PIN_SET){
			_icm20948Callback();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartDebug){

	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart == huartDebug){
		if(huart->RxEventType == HAL_UART_RXEVENT_IDLE){
			std::array<uint8_t, 5> startStr = {'s','t','a','r','t'};
			if(Size < 5){
				rxBuffer.fill(0);
				HAL_UARTEx_ReceiveToIdle_IT(huartDebug, rxBuffer.data(), rxBuffer.size());
				return;
			}
			for(uint8_t n=0; n<Size; n++){
				if(startStr[n] != rxBuffer[n]){
					rxBuffer.fill(0);
					HAL_UARTEx_ReceiveToIdle_IT(huartDebug, rxBuffer.data(), rxBuffer.size());
					return;
				}
			}
			// TODO:start handle
			__HAL_TIM_SET_COMPARE(ledTim, YELLOW_LED_CHANNEL, 500);
			imuDataCounter = 0;
			isMode2 = true;
			start = elapsedTimer->getTimeMS();
			_icm20948Callback = icm20948Callback2;

			rxBuffer.fill(0);
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartDebug){

	}
}


uint8_t transmitIMU(Vector3D<int16_t> &accel, Vector3D<int16_t> &gyro){
//	static std::array<uint8_t, 14> transmitBuffer;
//	static uint8_t counter = 0;
//
//	counter = counter++%16;
//
//	transmitBuffer[0] = (counter)|0xf0;
//	transmitBuffer[1] = 0xff;
//	for(uint8_t n=0; n<3; n++){
//		transmitBuffer[2*n+2] = (uint8_t)(accel[n]>>8);
//		transmitBuffer[2*n+3] = (uint8_t)(accel[n]&0xff);
//
//		transmitBuffer[2*n+8] = (uint8_t)(gyro[n]>>8);
//		transmitBuffer[2*n+9] = (uint8_t)(gyro[n]&0xff);
//	}
	static std::array<uint8_t, 13> transmitBuffer;
	static uint8_t counter = 0;

	counter = counter++%16;

//	transmitBuffer[0] = (counter)|0xf0;
	transmitBuffer[0] = 0x7f;
	for(uint8_t n=0; n<3; n++){
		transmitBuffer[2*n+1] = (uint8_t)(accel[n]>>8);
		transmitBuffer[2*n+2] = (uint8_t)(accel[n]&0xff);

		transmitBuffer[2*n+7] = (uint8_t)(gyro[n]>>8);
		transmitBuffer[2*n+8] = (uint8_t)(gyro[n]&0xff);
	}
	return HAL_UART_Transmit_DMA(huartDebug, (uint8_t *)transmitBuffer.data(), transmitBuffer.size());
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

}
