/*
 * Message.h
 *
 *  Created on: Dec 3, 2016
 *      Author: konradhalas
 */

#ifndef APPLICATION_USER_CORE_MESSAGE_H_
#define APPLICATION_USER_CORE_MESSAGE_H_

#include <string>

class Message {
public:
	virtual std::string serialize() = 0;
	virtual ~Message() {};
};

class ButtonSateMessage : public Message {
public:
	ButtonSateMessage(bool isUp): isUp(isUp) {};
	std::string serialize();

private:
	bool isUp;
};

class BatteryMessage : public Message {
public:
	BatteryMessage(int voltage): voltage(voltage) {};
	std::string serialize();

private:
	int voltage;
};

class MessgesSender {
public:
	virtual void send(Message *message) = 0;
	virtual ~MessgesSender() {};
};

#endif /* APPLICATION_USER_CORE_MESSAGE_H_ */
