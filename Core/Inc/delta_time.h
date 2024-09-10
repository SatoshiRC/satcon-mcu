/*
 * delta_time.h
 *
 *  Created on: Sep 9, 2024
 *      Author: ohya
 */

#ifndef INC_DELTA_TIME_H_
#define INC_DELTA_TIME_H_

#include "elapsedTimer/elapsedTimer.h"

struct DeltaTime{
	DeltaTime(ElapsedTimer *elapsedTimer = new ElapsedTimer())
	:elapsedTimer(elapsedTimer){}

	inline float getTimeMS(){
		return elapsedTimer->getTimeMS();
	}

	void update_dt(){
		uint64_t count = elapsedTimer->getCount();
		uint64_t delta = count - befCount;
		befCount = count;
		dt = delta / 1000000.0;
	}

	/*
	 * @brief : return the delta time in seconds.
	 * delta time will update in update_dt() only.
	 */
	float getDelta(){
		return dt;
	}
private:
	ElapsedTimer *elapsedTimer;
	uint64_t befCount = 0;
	float dt = 0;


};



#endif /* INC_DELTA_TIME_H_ */
