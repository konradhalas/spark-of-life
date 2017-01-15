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
		virtual void requestSensorsMeasure() = 0;
		virtual void setMotorSpeed(int motor, int direction, int speed) = 0;
		virtual int getMotorSetpointSpeed(int motor) = 0;

		void handleButtonStateChange(bool isUp);
		void handleBatteryVoltageMeasured(int voltage);
		void handleSensorsMeasured(int s1, int s2, int s3);
		void handleMotorSpeedMeasured(int motor, int value);

	private:
		MessgesSender *messagesSender;
};


#endif /* APPLICATION_USER_CORE_DEVICEMANAGER_H_ */
