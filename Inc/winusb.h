#ifndef WINUSB_H_INCLUDED
#define WINUSB_H_INCLUDED

#include "winusb_defs.h"

#define MS_OS_DESCRIPTOR_INDEX 0xEE

/* Arbitrary, but must be equivalent to the last character in the
 * special OS descriptor string (returned by request to string index 0xEE).
 * We use 0x21 (equivalent to ASCII code of '!' character)
 */
#define WINUSB_MS_VENDOR_CODE '!'

struct winusb_extended_property_device_interface_guid_descriptor {
    uint32_t dwLength;
    uint16_t bcdVersion;
    uint16_t wIndex;
    uint16_t wCount;
    uint32_t dwSize;
    uint32_t dwPropertyDataType;
    uint16_t wPropertyNameLength;
    const uint16_t bPropertyName[21];
    uint32_t dwPropertyDataLength;
    const uint16_t bPropertyData[];
} __attribute__((packed));

#endif
