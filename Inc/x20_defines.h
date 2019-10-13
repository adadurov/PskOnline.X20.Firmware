#ifndef X20_COMMANDS_INCLUDED
#define X20_COMMANDS_INCLUDED

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

	// 10 A textual description of the device's firmware version
	//    A zero-terminated ASCII string containing only numbers,
	//    Latin letters, dots and parentheses.
	//    For example: '1.2.3-alpha-29ab2fd9'
	uint8_t     firmware_build_date[30];

	// 40 A textual description of the device's firmware version
	//    A zero-terminated ASCII string containing only numbers,
	//    Latin letters, dots and parentheses.
	//    For example: '1.2.3-alpha-29ab2fd9'
	uint8_t     revision_info[];

}  __attribute__((packed))
x20_capabilities_descriptor;


#define TR_BUF_SAMPLE_T             uint32_t

typedef struct {
    // номер пакета после получения команды старт (32-бит)			4
    uint32_t package_number;

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

#endif
