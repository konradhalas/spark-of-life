/*
 * DeviceManager.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#include <DeviceManager.h>

void DeviceManager::handleButtonStateChange(bool isUp) {
	messagesSender->send(new ButtonSateMessage(isUp));
}

void DeviceManager::handleBatteryVoltageMeasured(int voltage) {
	messagesSender->send(new BatteryMessage(voltage));
}

void DeviceManager::handleSensorsMeasured(int s1, int s2, int s3) {
	messagesSender->send(new SensorsMessage(s1, s2, s3));
}
