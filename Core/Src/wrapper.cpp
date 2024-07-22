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

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 3;
void message(std::string str, uint8_t level = 3);



std::array<float, 3> gyroValue;
std::array<float, 3> AccelValue;

void init(){
	bool isInitializing = true;

	SET_MASK_ICM20948_INTERRUPT();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	std::bitset<8> bitset_1byte = 0;

	if(elapsedTimer->selfTest() == false){
		message("ERROR : elapsed timer freaquency is not correct",2);
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
	}catch(std::runtime_error &e){
		message("Error : Icm20948 is not detected",2);
	}
	icm20948User.init();
	CLEAR_MASK_ICM20948_INTERRUPT();
	message("ICM20948 is initialized");

	while(isInitializing){
		isInitializing = !attitudeEstimate.isInitialized();
	}
	message("Initialization is complete");

	esc.enable();
	esc.arm();
}

void loop(){

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_1){
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){
			message("ICM20948 interrupt",2);
			attitudeEstimate.updateTime();
			Vector3D<float> accel;
			Vector3D<float> gyro;
			icm20948->readIMU();
			icm20948->getAccel(accel);
			icm20948->getGyro(gyro);

			attitudeEstimate.getAccelValue() = accel;
			attitudeEstimate.getGyroValue() = gyro;

			attitudeEstimate.updateIMU();
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3){

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
		HAL_UART_Receive_DMA(huartSbus,hsbus.getReceiveBufferPtr(),hsbus.getDataLen());
	}
}

void message(std::string str, uint8_t level){
	if(level <= messageLevel){
		str += "\n";
		HAL_UART_Transmit_DMA(&huart3, (uint8_t *)str.c_str(), str.length());
	}
	return;
}
