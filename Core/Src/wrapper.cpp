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

/*
 * param:
 * param1(str):set message in std::string
 * param2(level):set message level. Planning to use message control.(For error = 0 and system message = 1, runtime message = 2, debug = 3)
 */
const uint8_t messageLevel = 0;
void message(std::string str, uint8_t level = 0);



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

	/*
	 * communication check with icm20948
	 * initialize icm20948
	 */
	uint8_t whoami = icm20948->whoami();;
	for(uint8_t n=0; n<10 && whoami!=0xea; n++){
		message("Error : Icm20948 is not detected \n retrying...",2);
		HAL_I2C_DeInit(&hi2c2);
		HAL_I2C_Init(&hi2c2);
		icm20948->changeUserBank(ICM20948::REGISTER::BANK::BANK0);
		icm20948->reset();
		HAL_Delay(100);
		whoami = icm20948->whoami();
	}
	if(whoami == 0xea){
		message("Icm20948 is detected",1);
		icm20948User.init();

		CLEAR_MASK_ICM20948_INTERRUPT();

	}else{
		message("Error : Icm20948 is not detected",1);
	}

	while(isInitializing){
		isInitializing = !attitudeEstimate.isInitialized();
	}
	message("Initialization is complete");
}

void loop(){


//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, 6, 100);
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

//	uint8_t tmp;
//	uint8_t pin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
//	HAL_I2C_Mem_Read(&hi2c2, ((uint8_t)ICM20948::Address::LOW)<<1, 26, 1, &tmp, 1, 100);
//	std::bitset<8> bitset_1byte = tmp;
//	message("Resister read " + std::to_string(26) + " : " + bitset_1byte.to_string() + ", INT pin state : " + std::to_string(pin),0);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == &hi2c2){
		message("I2C master rx complete callback");
		attitudeEstimate.updateIMU();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_1){
//		message("GPIO_EXIT");
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET){
//			message("Icm20948 interrupt");
			attitudeEstimate.updateTime();
			Vector3D<float> accel;
			Vector3D<float> gyro;
//			icm20948->get6ValueBurst(accel, gyro);
			icm20948->readIMU();
			icm20948->getAccel(accel);
			icm20948->getGyro(gyro);

			attitudeEstimate.getAccelValue() = accel;
			attitudeEstimate.getGyroValue() = gyro;

			attitudeEstimate.updateIMU();

			if(hdma_usart3_tx.State == HAL_DMA_StateTypeDef::HAL_DMA_STATE_READY){
				auto attitude = attitudeEstimate.getAttitude();
				const uint8_t bufSize = 18;
				int8_t buf[bufSize];
				buf[bufSize - 2] = '\r';
				buf[bufSize - 1] = '\n';
				for(uint8_t n=0; n<4; n++){
					buf[n] = std::roundf(attitude[n]*100.0);
				}
				auto rawAccel = icm20948->getRawAccel();
				for(uint8_t n=0; n<3; n++){
					buf[4 + n*2] = rawAccel.data()[n]>>8;
					buf[4 + n*2 + 1] = rawAccel.data()[n]>>8 & 0xff;
				}
				auto rawGyro = icm20948->getRawGyro();
				for(uint8_t n=0; n<3; n++){
					buf[10 + n*2] = rawGyro.data()[n]>>8;
					buf[10 + n*2 + 1] = rawGyro.data()[n]>>8 & 0xff;
				}
				HAL_UART_Transmit_DMA(&huart4, (uint8_t *)buf, sizeof(buf));
			}
		}else{

		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart3){

	}
}

void message(std::string str, uint8_t level){
	if(level >= messageLevel){
		str += "\n";
//		HAL_UART_Transmit_DMA(&huart3, (uint8_t *)str.c_str(), str.length());
	}
	return;
}
