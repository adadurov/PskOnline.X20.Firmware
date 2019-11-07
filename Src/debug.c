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

extern void trace_write_init(UART_HandleTypeDef *huart)
{
	debugUart = huart;
	debugCounter = 0;
}


void trace_write_string(const char* string)
{
	uint32_t len = 0;
	const char* pTermChar = string;
	while( *pTermChar != 0 )
	{
		++len;
		++pTermChar;
	}

	HAL_UART_Transmit(debugUart, string, len /* chars to transmit */, len /* milliseconds*/);
}

void trace_write_int(uint32_t value)
{
	char buffer[10];
	utoa(value, buffer, 16);
	buffer[8] = 0;
	trace_write_string(buffer);
}

void trace_write_counter()
{
	debugCounter += 1;
	trace_write_int(debugCounter);
	trace_write_string(": ");
}


void trace_write_newline()
{
	trace_write_string("\r\n");
}

void usb_debug_usb_setup_trace(const char *source, USBD_SetupReqTypedef *req)
{
  trace_write_counter();
  trace_write_string(source);
  trace_write_string(" ");
  trace_write_int(req->bmRequest);
  trace_write_string(" ");
  trace_write_int(req->bRequest);
  trace_write_string(" ");
  trace_write_int(req->wValue);
  trace_write_string(" ");
  trace_write_int(req->wIndex);
  trace_write_string(" ");
  trace_write_int(req->wLength);
  trace_write_newline();
}
