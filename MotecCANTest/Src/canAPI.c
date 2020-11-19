/*
 * canAPI.c
 *
 *  Created on: Oct 17, 2020
 *      Author: piotr
 */


#include "canAPI.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

CAN_HandleTypeDef hcan1;
uint32_t allReceiveIDs[NUMOFRXMESSAGES][2];
CANRXMessage allReceivedMessages[NUMOFRXMESSAGES];
uint16_t allReceivedMessageCount;


CAN_API_Status canAPIInit(CAN_HandleTypeDef hcan)
{
	hcan1 = hcan;
	//125kbps Pre = 24, SJW = 3, BS1 = 11, BS2 = 4
	//500kbps Pre = 6, SJW = 1, BS1 = 11, BS2 = 4
	//1Mbps  Pre = 3, SJW = 1, BS1 = 11, BS2 = 4
	hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;//6 3 4
	hcan1.Init.Mode = CAN_MODE_NORMAL;//CAN_MODE_LOOPBACK;//CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;//6 13 6
	hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;//1 2 5
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	while(HAL_CAN_Init(&hcan1) != HAL_OK)
	{
//		return CAN_API_STATUS_ERROR;
	}

	CAN_FilterTypeDef filter1;
	filter1.FilterIdHigh = 0x0000;//0x1fff;//0x1fff;
	filter1.FilterIdLow = 0x0000;//0xffff; //0xffff;
	filter1.FilterMaskIdHigh = 0x0000;
	filter1.FilterMaskIdLow = 0x0000;
	filter1.FilterFIFOAssignment = CAN_RX_FIFO0;
	filter1.FilterBank = 0;
	filter1.FilterMode = CAN_FILTERMODE_IDMASK;
	filter1.FilterScale = CAN_FILTERSCALE_16BIT;
	filter1.FilterActivation = ENABLE;
	filter1.SlaveStartFilterBank = 0;

	if(HAL_CAN_ConfigFilter(&hcan1, &filter1) != HAL_OK)
	{
		return CAN_API_STATUS_ERROR;
	}

	allReceivedMessageCount = 0;
	for(int i = 0; i < NUMOFRXMESSAGES; i++)
	{
		allReceiveIDs[i][0] = allRXMessages[i]->CANID;
		allReceiveIDs[i][1] = 0;
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		return CAN_API_STATUS_ERROR;
	}

	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		return CAN_API_STATUS_ERROR;
	}

	return CAN_API_STATUS_OK;
}

//TODO: Figure out returning status
//TODO: Handle sending RTRs if needed
//TODO: Handle figuring out how to abort messages
CAN_API_Status canAPIRun(void)
{
	//Send Messages
	for(int i = 0; i < NUMOFTXMESSAGES; i++)
	{
		if(hasTimedOut(allTXMessages[i]->sendRateMs, allTXMessages[i]->timeStamp))
		{
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t data[8];
			uint32_t dataLength;
			if(allTXMessages[i]->sendFunction != NULL)
			{
				allTXMessages[i]->sendFunction(data, &dataLength);
			}
			TxHeader.DLC = dataLength;
			TxHeader.StdId = allTXMessages[i]->CANID;
			TxHeader.IDE = allTXMessages[i]->idType;
			TxHeader.RTR = allTXMessages[i]->messageType;

			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
			{
				//TODO: Do something here? Keep status and return it later we do not want to delay other messages

			}

			while(allTXMessages[i]->timeoutMs != 0x0000 && hasTimedOut(allTXMessages[i]->timeoutMs, allTXMessages[i]->timeStamp) && HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox));
			if(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox) && allTXMessages[i]->timeoutMs != 0x0000)
			{
				HAL_CAN_AbortTxRequest(&hcan1, TxMailbox);
			}
			allTXMessages[i]->timeStamp = getTimeoutMS();
		}
	}

	//Receive Messages
	for(int i = 0; i < allReceivedMessageCount; i++)
	{
		for(int j = 0; j < NUMOFRXMESSAGES; j++)
		{
			if(allReceivedMessages[i].CANID == allRXMessages[j]->CANID)
			{
				allRXMessages[j]->idType = allReceivedMessages[i].idType;
				allRXMessages[j]->messageData = allReceivedMessages[i].messageData;
				allRXMessages[j]->messageLength = allReceivedMessages[i].messageLength;
				allRXMessages[j]->messageType = allReceivedMessages[i].messageType;
				if(allRXMessages[j]->receiveFunction != NULL)
				{
					allRXMessages[j]->receiveFunction(allRXMessages[j]->messageData);
				}
				allReceivedMessageCount--;
			}
		}
	}

	return CAN_API_STATUS_OK;
}

void handleCANReceiveInterrupt(void)
{
	CANRXMessage message;
	CAN_RxHeaderTypeDef RxMessage;
	uint8_t RxData[8];
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage, RxData);
	for(int i = 0; i < NUMOFRXMESSAGES; i++)
	{
		if(allReceiveIDs[i][0] == RxMessage.StdId)
		{
			allReceiveIDs[i][1] = 1;
			message.CANID = RxMessage.StdId;
			message.idType = RxMessage.IDE;
			message.messageData = RxData;
			message.messageLength = RxMessage.DLC;
			message.messageType = RxMessage.RTR;
			allReceivedMessages[allReceivedMessageCount] = message;
			allReceivedMessageCount++;
		}
	}
}

//TODO: Handle reading RTRs if needed
//TODO: Handle figuring out how keep this call faster
//TODO: We do not want to do alot of things in an interrupt handler
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1)
		handleCANReceiveInterrupt();
}
