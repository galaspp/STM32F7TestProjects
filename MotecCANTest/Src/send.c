/*
 * send.c
 *
 *  Created on: Oct 18, 2020
 *      Author: piotr
 */

#include "send.h"

void testSendingData(uint8_t * ptr, uint32_t * len)
{
	*len = 8;
	ptr[0] = 0x00;
	ptr[1] = 0xFF;
	ptr[2] = 0x00;
	ptr[3] = 0xFF;
	ptr[4] = 0x00;
	ptr[5] = 0xFF;
	ptr[6] = 0x00;
	ptr[7] = 0xFF;

}
