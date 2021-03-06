#ifndef PSK_X20_DEFINES
#define PSK_X20_DEFINES

// the number of bytes per sample
#define TR_BUF_SAMPLE_SIZE          sizeof(uint32_t)

typedef void* HX20_SENSOR;

// Generation 0
typedef struct
{
	// 0 the size of the entire structure
	uint16_t   	size;

	// 2 The generation number for the firmware & device.
	//   The clients may choose which generations they do support.
	uint16_t    generation;

	// 4 bits per sample
	uint16_t    bits_per_sample;

	// 6 sampling rate (Hz)
	uint16_t    sampling_rate;

	// 8 Bytes per transfer over PHYSIO_EPIN_ADDR endpoint
	uint16_t    bytes_per_physio_transfer;

	// 10 Index of the USB string descriptor representing the device's firmware build date
	uint16_t    firmware_build_date_str_idx;

	// 12 Index of the USB string descriptor representing the device's firmware revision
	uint16_t    revision_info_str_idx;

}  __attribute__((packed))
x20_capabilities;


typedef struct {
    // sequential number of the package                            4
	// (this is reset to 0 by each START command)
    uint32_t package_number;

    // bitmap with status flags (32 bit)                            4
    // reserved for future use
	uint32_t flags;

    // reserved (32 bit)                                            4
	uint32_t reserved;

	// the number of ring buffer overflows detected                 4
	// since the previous package had been transmitted
	// will not exceed INT32_MAX
    int32_t ring_buffer_overflows;

	// the number of samples remaining in the FIFO                  2
    uint16_t ring_buffer_data_count;

	// the number of 32-bit samples in the package                  2
	uint16_t num_samples;

	// flexible array of bytes representing the samples
	// each sample occupies 4 bytes
	uint8_t samples[];

} __attribute__((packed))
usb_package;

enum x20_commands {
  // returns the capabilities descriptor defined by
  // x20_capabilities_descriptor
  X20_GET_CAPABILITIES_DESCRIPTOR = 0x20,

  X20_USE_PPG = 0x30,

  X20_USE_RAMP = 0x31,

  X20_START = 0x50,

  X20_STOP = 0x51
};

HX20_SENSOR X20_ConfigureSensor(
		I2C_HandleTypeDef* phi2c,
		uint16_t max_usb_package_size,
		uint8_t (*usbFreeToTransmit)(),
		uint8_t (*usbTransmit)(uint8_t*, uint16_t),
		void (*ErrorHandler)()
		);

void X20_Task(HX20_SENSOR sensor);

void X20_Start(HX20_SENSOR sensor);

void X20_Stop(HX20_SENSOR sensor);

void X20_UseRamp(HX20_SENSOR sensor);

void X20_UsePpg(HX20_SENSOR sensor);

x20_capabilities* X20_GetCapabilities(HX20_SENSOR sensor);

uint8_t X20_IsStarted(HX20_SENSOR sensor);

#endif
