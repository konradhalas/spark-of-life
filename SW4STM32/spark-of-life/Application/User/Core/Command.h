/*
 * Command.h
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#ifndef APPLICATION_USER_CORE_COMMAND_H_
#define APPLICATION_USER_CORE_COMMAND_H_

#include "CommandsDispatcher.h"

class Command {
	public:
		virtual void dispatch(CommandsDispatcher *dispatcher) = 0;
		virtual ~Command() {}
};

class ToggleCommand: public Command {
	public:
		void dispatch(CommandsDispatcher *dispatcher);
};

class ReadSensorsCommand: public Command {
	public:
		void dispatch(CommandsDispatcher *dispatcher);
};

#endif /* APPLICATION_USER_CORE_COMMAND_H_ */
