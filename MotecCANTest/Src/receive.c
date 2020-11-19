/*
 * receive.c
 *
 *  Created on: Oct 18, 2020
 *      Author: piotr
 */

#include "receive.h"
#include "main.h"

void testReceiveData(uint8_t * ptr)
{
	//if(ptr[0] == 0x00 && ptr[1] == 0xFF)
		HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_14);
}
