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

void DeviceManager::handleBatteryVoltageMesured(int voltage) {
	messagesSender->send(new BatteryMessage(voltage));
}
