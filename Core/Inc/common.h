/*
 * common.h
 *
 *  Created on: Jul 5, 2024
 *      Author: ohya
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "ICM20948/ICM20948_USER.h"
#include "elapsedTimer/elapsedTimer.h"
#include "AttitudeEstimation.h"

#include "tim.h"

extern DMA_HandleTypeDef hdma_usart3_tx;

ICM20948_HAL *icm20948 = new ICM20948_HAL(&hi2c2, ICM20948::Address::LOW);
ICM20948_USER<ICM20948_HAL> icm20948User(icm20948);

ElapsedTimer *elapsedTimer = new ElapsedTimer(&htim5, 1000000);
AttitudeEstimation attitudeEstimate(elapsedTimer);

bool isInitializing = true;

#endif /* INC_COMMON_H_ */
