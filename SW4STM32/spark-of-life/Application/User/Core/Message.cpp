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
	stringStream << "BATTERY " << voltage;
	return stringStream.str();
}


std::string SensorsMessage::serialize() {
	std::ostringstream stringStream;
	stringStream << "SENSORS " << s1 << " " << s2 << " " << s3;
	std::string val = stringStream.str();
	return val;
}
