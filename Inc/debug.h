#ifndef __DEBUG_OUTPUT_H
#define __DEBUG_OUTPUT_H

#include "debug_control.h"

	extern void trace_write_counter();
	extern void trace_write_string(const char* string);
	extern void trace_write_int(unsigned int value);
	extern void trace_write_newline();
	extern void trace_write_init(UART_HandleTypeDef *huart);


#ifdef ENABLE_DEBUG

	#define debug_write_counter   trace_write_counter
    #define debug_write_string    trace_write_string
    #define debug_write_int       trace_write_int
    #define debug_write_newline   trace_write_newline

#else

	#define debug_write_counter()

	#define debug_write_string(s)
	#define debug_write_int(v)
	#define debug_write_newline()
	#define debug_write_init(huart)

#endif

#endif
