/*
 * canMessage.h
 *
 *  Created on: Oct 18, 2020
 *      Author: piotr
 */


#ifndef CANMESSAGE_H_
#define CANMESSAGE_H_

#define NUMOFTXMESSAGES 0
#define NUMOFRXMESSAGES 1

typedef void (*SendHandler)(uint8_t * ptr, uint32_t * len);
typedef void (*ReceiveHandler)(uint8_t * ptr);

typedef enum {
	CAN_ID_STANDARD = 0,
	CAN_ID_EXTENDED,
} CAN_ID_Type;

typedef enum {
	CAN_MESSAGE_DATA_FRAME= 0,
	CAN_MESSAGE_REMOTE_FRAME,
} CAN_MESSAGE_Type;

typedef struct CANTXMessage {
	uint32_t CANID;
	CAN_ID_Type idType;
	CAN_MESSAGE_Type messageType;
	uint32_t sendRateMs;
	uint32_t timeStamp;
	uint16_t timeoutMs;
	uint32_t messageLength;
	uint8_t* messageData;
	SendHandler sendFunction;
} CANTXMessage;

typedef struct CANRXMessage {
	uint32_t CANID;
	CAN_ID_Type idType;
	CAN_MESSAGE_Type messageType;
	uint32_t messageLength;
	uint8_t* messageData;
	ReceiveHandler receiveFunction;
} CANRXMessage;

extern CANTXMessage *allTXMessages[1];
extern CANRXMessage *allRXMessages[1];

#endif /* CANMESSAGE_H_ */
