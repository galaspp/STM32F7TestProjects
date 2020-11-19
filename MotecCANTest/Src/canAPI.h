/*
 * canAPI.h
 *
 *  Created on: Oct 17, 2020
 *      Author: piotr
 */

#ifndef CANAPI_H_
#define CANAPI_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "timeout.h"
#include "canMessage.h"


typedef enum {
	CAN_API_STATUS_OK = 0,
	CAN_API_STATUS_ERROR,

} CAN_API_Status;

CAN_API_Status canAPIInit(CAN_HandleTypeDef hcan);
CAN_API_Status canAPIRun(void);

#endif /* CANAPI_H_ */
