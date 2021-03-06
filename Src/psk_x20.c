#include "stm32f1xx_hal.h"

#include "psk_x20.h"
#include "ring_buffer.h"
#include "debug.h"
#include "max30102.h"
#include "stdlib.h"

#define USE_PPG 1
//#define USE_PPG 0

typedef struct {
    uint16_t physioTransferSize;

    uint16_t samplesPerPackage;

    uint16_t bitsPerSample;

    uint16_t samplingRate;

	uint8_t startFlipped;

	uint32_t startTicks;

	uint8_t stopFlipped;

	uint32_t stopTicks;

    uint8_t started;

	uint8_t usingPpg;

	uint32_t ramp;

	ring_buffer *pRingBuf;

	usb_package *transmit_buffer;

	I2C_HandleTypeDef *pHI2C;

	x20_capabilities capabilities;

	uint8_t (*usbTransmit)(uint8_t*, uint16_t);

	uint8_t (*usbFreeToTransmit)();

 } X20_SENSOR;


static X20_SENSOR singleSensor;

#define RING_BUFFER_SAMPLES         1024

HX20_SENSOR X20_ConfigureSensor(I2C_HandleTypeDef* phi2c, uint16_t max_usb_package_size, uint8_t (*usbFreeToTransmit)(), uint8_t (*usbTransmit)(uint8_t*, uint16_t), void (*ErrorHandler)() )
{
	const uint32_t samplesPerPackage = (max_usb_package_size - sizeof(usb_package)) / TR_BUF_SAMPLE_SIZE;
    const uint32_t actualTransferSize = sizeof(usb_package) + samplesPerPackage * TR_BUF_SAMPLE_SIZE;

	singleSensor.usbFreeToTransmit = usbFreeToTransmit;
	singleSensor.usbTransmit = usbTransmit;
    singleSensor.bitsPerSample = 18;
    singleSensor.samplingRate = 400;
    singleSensor.samplesPerPackage = samplesPerPackage;
    singleSensor.physioTransferSize = actualTransferSize;
    singleSensor.started = 0;
    singleSensor.usingPpg = USE_PPG;
    singleSensor.startFlipped = 0;
    singleSensor.stopFlipped = 0;
    singleSensor.stopTicks = 0;
    singleSensor.startTicks = 0;

    singleSensor.pHI2C = phi2c;

    // allocate memory
    singleSensor.pRingBuf = ring_buffer_alloc(RING_BUFFER_SAMPLES);

    usb_package *__transmit_buffer = (usb_package*)malloc(actualTransferSize + 16);

    // align to 4 bytes
    // you should not attempt to 'free' the pointer after doing this
    singleSensor.transmit_buffer = (usb_package *)(((long long int)__transmit_buffer) & 0xFFFFFFFFC);
    singleSensor.transmit_buffer->package_number = 0;

	if (0 == singleSensor.pRingBuf) {
		trace_write_string(".................failed to allocate ring buffer for ");
		trace_write_int(RING_BUFFER_SAMPLES);
		trace_write_newline();
		ErrorHandler();
	}
	if (0 == singleSensor.transmit_buffer) {
		trace_write_string(".................failed to allocate transmit_buffer of ");
		trace_write_int(max_usb_package_size);
		trace_write_newline();
		ErrorHandler();
	}

    HAL_StatusTypeDef max30102_status = MAX30102_Init(phi2c);
    trace_write_string("  MAX30102 part_id:     0x");

    if (HAL_OK == max30102_status)
    {
  	  uint8_t partId = MAX30102_GetPartId(phi2c);
  	  trace_write_int(partId);
  	  trace_write_newline();
    }
    else
    {
  	  trace_write_string("    Failed to configure. Status: ");
  	  trace_write_int(max30102_status);
  	  trace_write_newline();
    }
	trace_write_string("  Samples per package:  0x"); trace_write_int(samplesPerPackage); trace_write_newline();
	trace_write_string("  USB package size:     0x"); trace_write_int(singleSensor.physioTransferSize); trace_write_newline();

    singleSensor.capabilities.size = sizeof(x20_capabilities);
    singleSensor.capabilities.generation = 0;
    singleSensor.capabilities.bits_per_sample = singleSensor.bitsPerSample;
    singleSensor.capabilities.sampling_rate = singleSensor.samplingRate;
    singleSensor.capabilities.bytes_per_physio_transfer = singleSensor.physioTransferSize;

    return (HX20_SENSOR)&singleSensor;
}

void CleanUpPendingCommands(X20_SENSOR *pSensor)
{
  pSensor->startFlipped = 0;
  pSensor->stopFlipped = 0;
  pSensor->stopTicks = 0;
  pSensor->startTicks = 0;
}

void DoStartAndCleanup(X20_SENSOR *pSensor)
{
	ring_buffer_clear(pSensor->pRingBuf);
    pSensor->transmit_buffer->package_number = 0;
	pSensor->started = 1;
    CleanUpPendingCommands(pSensor);
}

void DoStopAndCleanup(X20_SENSOR *pSensor)
{
	ring_buffer_clear(pSensor->pRingBuf);
	pSensor->started = 0;
    CleanUpPendingCommands(pSensor);
}

void X20_ExecutePendingCommands(X20_SENSOR *pSensor)
{
	if (pSensor->startFlipped != 0 && pSensor->stopFlipped != 0)
	{
		int startPriority = pSensor->startTicks > pSensor->stopTicks ? 1 : 0;
		if (0 != startPriority)
		{
			debug_write_string("Executed START with priority."); debug_write_newline();
			DoStartAndCleanup(pSensor);
		}
		else
		{
			debug_write_string("Executed STOP with priority."); debug_write_newline();
			DoStopAndCleanup(pSensor);
		}
		return;
	}
	else if (pSensor->startFlipped != 0)
	{
        debug_write_string("Executed START."); debug_write_newline();
        DoStartAndCleanup(pSensor);
	}
	else if (pSensor->stopFlipped != 0)
	{
        debug_write_string("Executed STOP."); debug_write_newline();
        DoStopAndCleanup(pSensor);
	}
}

void X20_Start(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;

  HAL_StatusTypeDef max30102_status = MAX30102_Init(pSensor->pHI2C);

  pSensor->startFlipped = 1;
  pSensor->startTicks = HAL_GetTick();
}

void X20_Stop(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;

  pSensor->stopFlipped = 1;
  pSensor->stopTicks = HAL_GetTick();
}

void X20_UseRamp(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;

  pSensor->usingPpg = 0;
}

void X20_UsePpg(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;

  pSensor->usingPpg = USE_PPG;
}

uint8_t X20_IsStarted(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;
  return pSensor->started;
}

x20_capabilities* X20_GetCapabilities(HX20_SENSOR hSensor)
{
  X20_SENSOR *pSensor = (X20_SENSOR *)hSensor;

  return &pSensor->capabilities;
}

void X20_PutSamplesToRingBuffer(X20_SENSOR *pSensor)
{
    int16_t availableSamples = MAX30102_GetNumSamplesInFifo(pSensor->pHI2C);

    if (pSensor->started != 0 && availableSamples > 0)
    {
        debug_write_string("==> Z: "); debug_write_int(availableSamples); debug_write_newline();
    }

    uint8_t sample[6];
    uint32_t value;

    for (int16_t i = 0; i < availableSamples; ++i)
    {
        MAX30102_ReadFifo(pSensor->pHI2C, sample, 6);

		  if (pSensor->started != 0)
		  {
			  if (pSensor->usingPpg)
			  {
		          value = ((sample[3] << 16) & 0x03) + (sample[4] << 8) + sample[5];
				  debug_write_string(" IR: "); debug_write_int(value); debug_write_newline();

			  }
			  else
			  {
				value = ++pSensor->ramp;
    	        debug_write_string(" RAMP: "); debug_write_int(value); debug_write_newline();
			  }

	          // put the value to our circular buffer for transmitting via USB
            ring_buffer_add_sample(pSensor->pRingBuf, value);
		  }
    }
}

void X20_TransmitSamples(X20_SENSOR *pSensor)
{
    if ( ! pSensor->usbFreeToTransmit() )
    {
//    	trace_write_string("_");  trace_write_newline();
        return;
    }

    ring_buffer *pRingBuf = pSensor->pRingBuf;
    usb_package *transmit_buffer = pSensor->transmit_buffer;
    uint16_t required_samples = pSensor->samplesPerPackage;

    uint16_t ring_buffer_samples = ring_buffer_get_count(pRingBuf);
    if (ring_buffer_samples >= required_samples)
    {
        // copy samples from the ring buffer to the transmit buffer
        for( uint16_t i = 0; i < required_samples; ++i)
        {
        	uint32_t sample = ring_buffer_remove_sample(pRingBuf);
        	uint32_t *buffer = transmit_buffer->samples;

            buffer[i] = sample;
	    }
	    transmit_buffer->package_number++;
	    transmit_buffer->reserved = pRingBuf->first;
	    // number of samples remaining in the ring buffer
	    transmit_buffer->ring_buffer_data_count = ring_buffer_get_count(pRingBuf);
	    transmit_buffer->num_samples = required_samples;
	    transmit_buffer->ring_buffer_overflows = pRingBuf->overflows;
	    // reset overflow counter
	    pRingBuf->overflows = 0;

	    uint16_t len = sizeof(usb_package) + TR_BUF_SAMPLE_SIZE * required_samples;

	    // put the package to the USB transfer buffer
	    // the USB state machine will transmit the buffer to the USB Host
	    // once the USB Host sends an IN token for the endpoint
        int start_tr = HAL_GetTick();
        int result = pSensor->usbTransmit((uint8_t*)transmit_buffer, len);
        int time = HAL_GetTick() - start_tr;

//        if (result == 0)
//        {
////            return USBD_OK == 0;
//        	trace_write_string(" T");
//        }
//        else if (result == 1)
//        {
////            return USBD_BUSY == 1;
//        	trace_write_string(" B");
//        }
//        else if (result == 2)
//        {
////            return USBD_FAIL == 2;
//        	trace_write_string(" F");
//        }
//    	trace_write_int(transmit_buffer->package_number);
//    	trace_write_newline();

        debug_write_string("TR_TIME: "); debug_write_int(time); debug_write_newline();
    }
}

void X20_Task(HX20_SENSOR hSensor)
{
    X20_SENSOR *pSensor = (X20_SENSOR*)hSensor;

    X20_ExecutePendingCommands(pSensor);

	uint16_t added = ring_buffer_get_count(pSensor->pRingBuf);
    {
	    X20_PutSamplesToRingBuffer(pSensor);

	    uint16_t now = ring_buffer_get_count(pSensor->pRingBuf);
		added = now - added;
    }


    {
		uint16_t transfrerred = ring_buffer_get_count(pSensor->pRingBuf);
	
		X20_TransmitSamples(pSensor);
	
		uint16_t remaining = ring_buffer_get_count(pSensor->pRingBuf);
		transfrerred -= remaining;
		if (pSensor->started && (0 != added || 0 != transfrerred))
		{
			debug_write_int(added);
			debug_write_string(" , ");
			debug_write_int(transfrerred);
			debug_write_string(" , ");
			debug_write_int(remaining);
			debug_write_newline();
		}
    }

}
