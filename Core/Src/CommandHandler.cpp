/*
 * CommandHandler.cpp
 *
 *  Created on: Aug 6, 2024
 *      Author: ohya
 */

#include <CommandHandler.h>

namespace multicopter {

/*
 * command protocol
 * [0] start byte : 0x73 ('s')
 * [1] command id
 * [2]-[n-2] command field
 * [n-1][n] last bytes : 0x0d 0x0c (\r\n)
 *
 * define the size of command field in source code.
 * In the rx event callback, call rxEvent() to check that if the data has received last bytes.
 * Once the last bytes (\r\n) is detected, __isReceivedEOL will be set.
 */




template<std::size_t __size>
COMMAND_HANDLER<__size>::COMMAND_HANDLER() {
	// TODO Auto-generated constructor stub
	__isReceivedEOL = false;
	it = buffer.begin();
}

template<std::size_t __size>
void COMMAND_HANDLER<__size>::clear(){
	__isReceivedEOL = false;
	buffer.fill(0);
}

template<std::size_t __size>
void COMMAND_HANDLER<__size>::rxEvent(){
	if(__isReceivedEOL == true){
		return;
	}

	if(*it == '\n'){
		if(*(it-1) == '\r'){
			__isReceivedEOL = true;
			command.id = buffer[1];

			auto bufIt = buffer.begin()+2;
			auto fieldIt = command.field.begin();
			for(uint8_t n=0;n<command.field.size();n++){
				//TODO:debug
				*fieldIt++ = *bufIt++;
			}
		}
	}

	if(++it>=buffer.end()){
		it = buffer.end()-1;
	}
}

} /* namespace multicopter */
