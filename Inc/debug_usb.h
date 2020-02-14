#ifndef __DEBUG_USB_OUTPUT_H
#define __DEBUG_USB_OUTPUT_H

#include "debug_control.h"

#ifdef ENABLE_USB_DEBUG

#include "usbd_def.h"
	extern void usb_debug_usb_setup_trace(const char *source, USBD_SetupReqTypedef *req);

	#define usb_debug_write_counter					debug_write_counter
	#define usb_debug_write_string					debug_write_string
	#define usb_debug_write_int						debug_write_int
	#define usb_debug_write_newline					debug_write_newline

#else

	#define usb_debug_usb_setup_trace(source, req)

	#define usb_debug_write_counter()

	#define usb_debug_write_string(s)
	#define usb_debug_write_int(v)
	#define usb_debug_write_newline()

#endif

#endif
