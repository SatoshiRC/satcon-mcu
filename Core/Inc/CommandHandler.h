/*
 * CommandHandler.h
 *
 *  Created on: Aug 6, 2024
 *      Author: ohya
 */

#ifndef INC_COMMANDHANDLER_H_
#define INC_COMMANDHANDLER_H_

#include <array>
#include <cstdint>

namespace multicopter {

template<std::size_t __size>
struct COMMAND{
	uint8_t id;
	std::array<uint8_t, __size> field;

	COMMAND(){
		id = 0;
		field = std::array<uint8_t, __size>();
	}
};

template<std::size_t __size>
class COMMAND_HANDLER {
public:
	COMMAND_HANDLER();
	void clear();
	void rxEvent();
	bool isReceivedEOL();

	uint8_t *getBufferPtr(){
		return (uint8_t*)buffer.data();
	}

	std::size_t getMaximumSize(){
		return __size;
	}
private:
	std::array<uint8_t, __size> buffer;
	typename std::array<uint8_t, __size>::iterator it;
	bool __isReceivedEOL;
	COMMAND<__size-4> command;
};

} /* namespace multicopter */

#endif /* INC_COMMANDHANDLER_H_ */
