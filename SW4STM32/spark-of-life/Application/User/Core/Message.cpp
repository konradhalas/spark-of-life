/*
 * Message.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#include <sstream>
#include <Message.h>

std::string ButtonSateMessage::serialize() {
	std::ostringstream stringStream;
	stringStream << "BUTTON " << (isUp ? "UP" : "DOWN");
	return stringStream.str();
}

std::string BatteryMessage::serialize() {
	std::ostringstream stringStream;
	stringStream << "BATERRY " << voltage;
	return stringStream.str();
}
