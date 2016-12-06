/*
 * CommandsDispatcher.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#include <CommandsDispatcher.h>

void CommandsDispatcher::dispatch(ToggleCommand *command) {
	deviceManager->toggleLed();
}

void CommandsDispatcher::dispatch(BatteryReadCommand *command) {
	deviceManager->requestBatteryMeasure();
}
