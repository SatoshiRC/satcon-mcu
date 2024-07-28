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

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
static void message(std::string str, uint8_t level = 3);
static std::string messageBuffer;

std::array<float, 3> gyroValue;
std::array<float, 3> AccelValue;

void init(){
	bool isInitializing = true;

	SET_MASK_ICM20948_INTERRUPT();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	if(elapsedTimer->selfTest() == false){
//		std::string tmp = "ERROR : elapsed timer freaquency is not correct";
//		message(&tmp,2);
		message("ERROR : elapsed timer freaquency is not correct",0);
	}
	elapsedTimer->start();

	//start to receive sbus.
	HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());

	/*
	 * communication check with icm20948
	 * initialize icm20948
	 */
	try{
		icm20948User.confirmConnection();
		message("ICM20948 is detected",2);
	}catch(std::runtime_error &e){
		message("Error : Icm20948 is not detected",0);
	}
	icm20948User.init();
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	while(isInitializing){
		isInitializing = !attitudeEstimate.isInitialized();
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1));

	}
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	message("Initialization is complete",1);

	esc.enable();
	esc.arm();
}

void loop(){
	HAL_Delay(30);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_1){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){
			message("ICM20948 interrupt",2);
			attitudeEstimate.updateTime();
			Vector3D<float> accel;
			Vector3D<float> gyro;
			icm20948->readIMU();
			icm20948->getIMU(accel, gyro);

			attitudeEstimate.setAccelValue(accel);
			attitudeEstimate.setGyroValue(gyro);
			attitudeEstimate.updateIMU();

			auto attitude = attitudeEstimate.getAttitude();
			auto z_machienFrame = attitude.rotateVector({0,0,1.0});
			float roll = std::asin(z_machienFrame[0]);
			float pitch = std::asin(z_machienFrame[1]);
			float yawRate = attitudeEstimate.getYawRate();

			multicopterInput.roll = roll;
			multicopterInput.pitch = pitch;
			multicopterInput.yawRate = yawRate;
			esc.setSpeed(hmulticopter->controller(multicopterInput));

		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == huartSbus){
		hsbus.onReceive(multicopterInput);
		if(hsbus.getData().failsafe){
			hmulticopter->rcFailSafe();
		}else if(hsbus.getData().framelost){
			hmulticopter->rcFrameLost();
		}
		multicopterInput.sbusPitchNorm = hsbus.getPitchNorm();
		multicopterInput.sbusRollNorm = hsbus.getRollNorm();
		multicopterInput.sbusYawRateNorm = hsbus.getYawNorm();
		multicopterInput.sbusAltitudeNorm = hsbus.getAltitudeNorm();
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
		
		esc.setSpeed(hmulticopter->controller(multicopterInput));
	}
}

static void message(std::string str, uint8_t level){
	if(level <= messageLevel){
		str += "\n";
		if(huart3.gState == HAL_UART_STATE_READY){
			messageBuffer = std::string(str);
			HAL_UART_Transmit_DMA(&huart3, (uint8_t *)messageBuffer.c_str(), messageBuffer.length());
		}
	}
	return;
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart3){
//		huart->gState = HAL_UART_STATE_READY;
//	}
//}
