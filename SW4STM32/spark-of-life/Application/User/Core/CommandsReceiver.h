#ifndef APPLICATION_USER_CORE_COMMANDSRECEIVER_H_
#define APPLICATION_USER_CORE_COMMANDSRECEIVER_H_

#include <stdint.h>

#include "CommandsDispatcher.h"
#include "Message.h"

class CommandsReceiver {
	public:
		CommandsReceiver(CommandsDispatcher *dispatcher) : dispatcher(dispatcher) {};
		void handleReceivedData(const uint8_t *data);
	private:
		CommandsDispatcher *dispatcher;
};

#endif /* APPLICATION_USER_CORE_COMMANDSRECEIVER_H_ */
