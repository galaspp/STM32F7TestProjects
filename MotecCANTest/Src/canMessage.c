/*
 * canMessage.c
 *
 *  Created on: Oct 18, 2020
 *      Author: piotr
 */
#include <stdint.h>
#include "canMessage.h"
#include "send.h"
#include "receive.h"

//typedef struct CANTXMessage {
//	uint32_t CANID;
//	CAN_ID_Type idType;
//	CAN_MESSAGE_Type messageType;
//	uint32_t sendRateMs;
//	uint16_t timeoutMs;
//	uint32_t messageLength;
//	uint8_t* messageData;
//	SendHandler sendFunction;
//} CANTXMessage;

static CANTXMessage testTXMessage = {
		.CANID = 0x1D0,
		.idType = CAN_ID_STANDARD,
		.messageType = CAN_MESSAGE_DATA_FRAME,
		.sendRateMs = 500,
		.timeStamp = 0,
		.timeoutMs = 0x0000,
		.sendFunction = &testSendingData,
};


//typedef struct CANRXMessage {
//	uint32_t CANID;
//	CAN_ID_Type idType;
//	CAN_MESSAGE_Type messageType;
//	uint32_t messageLength;
//	uint8_t* messageData;
//	ReceiveHandler receiveFunction;
//} CANRXMessage;

static CANRXMessage testRXMessage = {
		.CANID = 0x1D0,
		.receiveFunction = &testReceiveData,
};


CANTXMessage *allTXMessages[1] = {
		[0] = &testTXMessage,
};

CANRXMessage *allRXMessages[1] = {
		[0] = &testRXMessage,
};
