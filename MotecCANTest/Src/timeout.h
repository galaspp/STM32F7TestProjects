/*
 * timeout.h
 *
 *  Created on: Oct 12, 2020
 *      Author: piotr
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define HEARTBEATTIME 500

void initTimeout(TIM_HandleTypeDef * htim2);
uint32_t getTimeoutMS(void);
bool addTimeoutMS(uint32_t addMS);
void clearTimeutMS(void);
bool hasTimedOut(uint32_t timeoutLimit, uint32_t startTime);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void handleHeartbeatLED(GPIO_TypeDef * GPIO, uint16_t pin);

#endif /* TIMEOUT_H_ */
