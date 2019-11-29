#ifndef PSK_X20_DEFINES
#define PSK_X20_DEFINES

// Generation 0
// revision info offset is 40 (decimal)
//
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

	// 10 Index of the string descriptor representing the device's firmware build date
	uint16_t    firmware_build_date_str_idx;

	// 12 Index of the string descriptor representing the device's firmware version
	uint16_t    revision_info_str_idx;

}  __attribute__((packed))
x20_capabilities_descriptor;


#define TR_BUF_SAMPLE_T             uint32_t

typedef struct {
    // номер пакета после получения команды старт (32-бит)			4
    uint32_t package_number;

    // bitmap with status flags (32 bit)                            4
	uint32_t flags;

    // die temperature                                              2
	// 8 higher-order bits - whole degrees in 2’s complement format
	// 4 low bits - fractions of 0.0625 degree (always positive)
	uint16_t die_temperature;

	// reserved (16 bit)                                            2
	uint16_t reserved;

	// количество переполнений кругового буфера (32 бит)			4
    uint32_t ring_buffer_overflows;

	// количество измерений в круговом буфере (16 бит)				2
    uint16_t ring_buffer_data_count;

	// количество следующих дальше 32-битных сэмплов (16 бит)		2
	uint16_t num_samples;

	// flexible array of samples
	TR_BUF_SAMPLE_T samples[];
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


enum x20_strings {
  X20_REVISION_STRING_IDX = 0x41,

  X20_BUILD_DATE_STRING_IDX = 0x42
};

#endif
