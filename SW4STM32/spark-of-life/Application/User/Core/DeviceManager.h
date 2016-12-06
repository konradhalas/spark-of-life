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
		virtual void requestBatteryMeasure() = 0;

		void handleButtonStateChange(bool isUp);
		void handleBatteryVoltageMeasured(int voltage);

	private:
		MessgesSender *messagesSender;
};


#endif /* APPLICATION_USER_CORE_DEVICEMANAGER_H_ */
