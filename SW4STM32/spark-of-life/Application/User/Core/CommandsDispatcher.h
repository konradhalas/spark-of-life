/*
 * CommandsDispatcher.h
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#ifndef APPLICATION_USER_CORE_COMMANDSDISPATCHER_H_
#define APPLICATION_USER_CORE_COMMANDSDISPATCHER_H_

#include "DeviceManager.h"

class ToggleCommand;
class BatteryReadCommand;

class CommandsDispatcher {
	public:
		CommandsDispatcher(DeviceManager *device): deviceManager(device) {}
		void dispatch(ToggleCommand *command);
		void dispatch(BatteryReadCommand *command);

	private:
		DeviceManager *deviceManager;
};

#endif /* APPLICATION_USER_CORE_COMMANDSDISPATCHER_H_ */
