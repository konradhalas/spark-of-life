/*
 * Command.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#include <Command.h>

void ToggleCommand::dispatch(CommandsDispatcher *dispatcher) {
	dispatcher->dispatch(this);
}

void ReadSensorsCommand::dispatch(CommandsDispatcher *dispatcher) {
	dispatcher->dispatch(this);
}
