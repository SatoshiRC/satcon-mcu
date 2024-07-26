/*
 * ICM20948_USER.h
 *
 *  Created on: Feb 10, 2023
 *      Author: OHYA Satoshi
 */

#ifndef ICM20948_ICM20948_USER_H_
#define ICM20948_ICM20948_USER_H_

#include "ICM20948/ICM20948_HAL.h"
#include "stdexcept"


class ICM20948_USER{
public:
	ICM20948_USER(ICM20948_HAL *icm20948):icm20948(icm20948){}

	void confirmConnection();
	void init();

	void update(){};

private:
	ICM20948_HAL *icm20948;
};

#endif /* ICM20948_ICM20948_USER_H_ */
