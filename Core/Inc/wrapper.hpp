/*
 * wrapper.hpp
 *
 *  Created on: Oct 23, 2023
 *      Author: ohya
 */


#ifndef INC_WRAPPER_HPP_
#define INC_WRAPPER_HPP_

#ifdef __cplusplus
extern "C" {
#endif

void init(void);
void loop(void);

void debug();

#ifdef __cplusplus
};
#endif

#endif /* INC_WRAPPER_HPP_ */

