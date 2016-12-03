/*
 * DeviceManager.h
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#ifndef APPLICATION_USER_CORE_DEVICEMANAGER_H_
#define APPLICATION_USER_CORE_DEVICEMANAGER_H_

#include "Message.h"

class DeviceManager {
	public:
		DeviceManager(MessgesSender *messagesSender): messagesSender(messagesSender) {}
		virtual ~DeviceManager() {};

		virtual void toggleLed() = 0;
		void handleButtonStateChange(bool isUp);
		void handleBatteryVoltageMesured(int voltage);

	private:
		MessgesSender *messagesSender;
};


#endif /* APPLICATION_USER_CORE_DEVICEMANAGER_H_ */
