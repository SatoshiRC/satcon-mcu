/*
 * common.h
 *
 *  Created on: Jul 5, 2024
 *      Author: ohya
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "ICM20948_USER.h"
#include "delta_time.h"
#include "Vector3D/Vector3D.h"
#include <functional>


#include "TWO_DOF_PID.h"
#include "HighPassFilter.h"

#include "tim.h"
#include "usart.h"

#include <array>

bool isInitializing = true;

#define RED_LED ledTim, TIM_CHANNEL_1
#define YELLOW_LED ledTim, TIM_CHANNEL_2
#define BLUE_LED ledTim, TIM_CHANNEL_3
uint16_t RED_LED_CHANNEL=TIM_CHANNEL_1;
uint16_t YELLOW_LED_CHANNEL=TIM_CHANNEL_2;
uint16_t BLUE_LED_CHANNEL=TIM_CHANNEL_3;
TIM_HandleTypeDef *ledTim = &htim1;

inline float degToRad(float deg){
	return deg*std::numbers::pi / 180.0;
}

inline float RadToDeg(float rad){
	return rad/std::numbers::pi * 180.0;
}

static std::function<void(void)> _icm20948Callback = [](){};

ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c2, ICM20948::Address::LOW);
ICM20948_USER icm20948User(icm20948);
void icm20948CallbackCalibration();
void icm20948Callback();
void icm20948Callback2();

ElapsedTimer *elapsedTimer = new ElapsedTimer(&htim5, 1000000);
DeltaTime *deltaTimer = new DeltaTime(elapsedTimer);

uint32_t adcValue = 0;

#endif /* INC_COMMON_H_ */
