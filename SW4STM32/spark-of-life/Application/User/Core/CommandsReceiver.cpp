/*
 * RemoteClient.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#include <CommandsReceiver.h>
#include <stdio.h>
#include <string>

#include "Command.h"


void CommandsReceiver::handleReceivedData(const uint8_t *data) {
	std::string dataAsStr((char*) data);
	Command *command = NULL;
	if (dataAsStr.find_first_of("TOGGLE") == 0) {
		command = new ToggleCommand();
	} else if (dataAsStr.find_first_of("READ_SENSORS") == 0) {
		command = new ReadSensorsCommand();
	}
	if (command) {
		command->dispatch(this->dispatcher);
	}
}
