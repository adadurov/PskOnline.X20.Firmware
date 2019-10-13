/*
 * debug.c
 *
 *  Created on: 08.03.2019
 *      Author: Alexey Adadurov
 */

#include "stm32f1xx_hal.h"
#include "usbd_def.h"

#include <stdlib.h>

static UART_HandleTypeDef *debugUart;
static unsigned int debugCounter;

extern void debug_write_init(UART_HandleTypeDef *huart)
{
	debugUart = huart;
	debugCounter = 0;
}


void debug_write_string(const char* string)
{
	while ( 0 != *string)
	{
		HAL_UART_Transmit(debugUart, string, 1, 2);
		string += 1;
	}
}

void debug_write_int(uint32_t value)
{
	char buffer[10];
	utoa(value, buffer, 16);
	buffer[8] = 0;
	debug_write_string(buffer);
}

void debug_write_counter()
{
	debugCounter += 1;
	debug_write_int(debugCounter);
	debug_write_string(": ");
}


void debug_write_newline()
{
	debug_write_string("\r\n");
}

void debug_usb_setup_trace(const char *source, USBD_SetupReqTypedef *req)
{
  debug_write_counter();
  debug_write_string(source);
  debug_write_string(" ");
  debug_write_int(req->bmRequest);
  debug_write_string(" ");
  debug_write_int(req->bRequest);
  debug_write_string(" ");
  debug_write_int(req->wValue);
  debug_write_string(" ");
  debug_write_int(req->wIndex);
  debug_write_string(" ");
  debug_write_int(req->wLength);
  debug_write_newline();
}
