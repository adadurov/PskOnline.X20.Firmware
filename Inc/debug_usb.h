#ifndef __DEBUG_USB_OUTPUT_H
#define __DEBUG_USB_OUTPUT_H


#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG

	extern void debug_usb_setup_trace(const char *source, USBD_SetupReqTypedef *req);

#else

	#define debug_usb_setup_trace(source, req)

#endif

#endif
