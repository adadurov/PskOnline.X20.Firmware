#ifndef __DEBUG_OUTPUT_H
#define __DEBUG_OUTPUT_H


#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG

	extern void debug_write_counter();
	extern void debug_write_string(const char* string);
	extern void debug_write_int(unsigned int value);
	extern void debug_write_newline();
	extern void debug_write_init(UART_HandleTypeDef *huart);

#else

	#define debug_write_counter()

	#define debug_write_string(s)
	#define debug_write_int(v)
	#define debug_write_newline()
	#define debug_write_init(huart)

#endif

#endif
