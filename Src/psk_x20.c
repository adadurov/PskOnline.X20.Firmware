#include "stm32f1xx_hal.h"

#include "psk_x20.h"
#include "ring_buffer.h"
#include "debug.h"


typedef struct {
    uint16_t physioTransferSize;

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

 } X20_SENSOR;


static X20_SENSOR sensor;

#define RING_BUFFER_SAMPLES         1024

HX20_SENSOR X20_ConfigureSensor(I2C_HandleTypeDef* phi2c2, uint16_t usb_package_size)
{
    sensor.bitsPerSample = 18;
    sensor.samplingRate = 400;
    sensor.physioTransferSize = usb_package_size;
    sensor.started = 0;
    sensor.usingPpg = 1;
    sensor.startFlipped = 0;
    sensor.stopFlipped = 0;
    sensor.stopTicks = 0;
    sensor.startTicks = 0;

    sensor.pHI2C = phi2c2;

    sensor.pRingBuf = ring_buffer_alloc(RING_BUFFER_SAMPLES);

    usb_package *__transmit_buffer = (usb_package*)malloc(usb_package_size);

    // align to 4 bytes
    sensor.transmit_buffer = (usb_package *)(((long long int)__transmit_buffer) & 0xFFFFFFFFC);
    sensor.transmit_buffer->package_number = 0;

	if (0 == sensor.pRingBuf) {
		trace_write_string(".................failed to allocate ring buffer for ");
		trace_write_int(RING_BUFFER_SAMPLES);
		trace_write_newline();
		Error_Handler();
	}
	if (0 == sensor.transmit_buffer) {
		trace_write_string(".................failed to allocate transmit_buffer of ");
		trace_write_int(usb_package_size);
		trace_write_newline();
		Error_Handler();
	}

    HAL_StatusTypeDef max30102_status = MAX30102_Init(phi2c2);
    trace_write_string("  MAX30102 part_id:   ");

    if (HAL_OK == max30102_status)
    {
  	  uint8_t partId = MAX30102_GetPartId(phi2c2);
  	  trace_write_int(partId);
  	  trace_write_newline();
    }
    else
    {
  	  trace_write_string("    Failed to configure. Status: ");
  	  trace_write_int(max30102_status);
  	  trace_write_newline();
    }


    sensor.capabilities.size = sizeof(x20_capabilities);
    sensor.capabilities.generation = 0;
    sensor.capabilities.bits_per_sample = sensor.bitsPerSample;
    sensor.capabilities.sampling_rate = sensor.samplingRate;
    sensor.capabilities.bytes_per_physio_transfer = sensor.physioTransferSize;
    sensor.capabilities.firmware_build_date_str_idx = X20_BUILD_DATE_STRING_IDX;
    sensor.capabilities.revision_info_str_idx = X20_REVISION_STRING_IDX;

    return (HX20_SENSOR)&sensor;
}

void CleanUpPendingCommands(X20_SENSOR *sensor)
{
  sensor->startFlipped = 0;
  sensor->stopFlipped = 0;
  sensor->stopTicks = 0;
  sensor->startTicks = 0;
}

void X20_ExecutePendingCommands(X20_SENSOR *sensor)
{
	if (sensor->startFlipped != 0 && sensor->stopFlipped != 0)
	{
		int startPriority = sensor->startTicks > sensor->stopTicks ? 1 : 0;
		if (startPriority != 0)
		{
			debug_write_string("Executed START with priority."); debug_write_newline();
			ring_buffer_clear(sensor->pRingBuf);
			sensor->started = 1;
		}
		else
		{
			debug_write_string("Executed STOP with priority."); debug_write_newline();
			ring_buffer_clear(sensor->pRingBuf);
			sensor->started = 0;
		}
		CleanUpPendingCommands(sensor);
		return;
	}
	if (sensor->startFlipped != 0)
	{
        debug_write_string("Executed START."); debug_write_newline();
        ring_buffer_clear(sensor->pRingBuf);
        sensor->started = 1;
        CleanUpPendingCommands(sensor);
        return;
	}
	if (sensor->stopFlipped != 0)
	{
        debug_write_string("Executed STOP."); debug_write_newline();
        ring_buffer_clear(sensor->pRingBuf);
        sensor->started = 0;
        CleanUpPendingCommands(sensor);
        return;
	}
}

void X20_Start(HX20_SENSOR hSensor)
{
  X20_SENSOR *sensor = (X20_SENSOR *)hSensor;

  HAL_StatusTypeDef max30102_status = MAX30102_Init(sensor->pHI2C);

  sensor->startFlipped = 1;
  sensor->startTicks = HAL_GetTick();
}

void X20_Stop(HX20_SENSOR hSensor)
{
  X20_SENSOR *sensor = (X20_SENSOR *)hSensor;

  sensor->stopFlipped = 1;
  sensor->stopTicks = HAL_GetTick();
}

void X20_UseRamp(HX20_SENSOR hSensor)
{
  X20_SENSOR *sensor = (X20_SENSOR *)hSensor;

  sensor->usingPpg = 0;
}

void X20_UsePpg(HX20_SENSOR hSensor)
{
  X20_SENSOR *sensor = (X20_SENSOR *)hSensor;

  sensor->usingPpg = 1;
}

uint8_t X20_IsStarted(HX20_SENSOR hSensor)
{
  X20_SENSOR *sensor = (X20_SENSOR *)hSensor;
  return sensor->started;
}

void X20_Task(HX20_SENSOR hSensor)
{
    X20_SENSOR *sensor = (X20_SENSOR *)hSensor;

    X20_ExecutePendingCommands(sensor);

    X20_PutSamplesToRingBuffer(sensor);

    X20_TransmitSamples(sensor, TR_BUF_SAMPLES);
}

void X20_PutSamplesToRingBuffer(X20_SENSOR *pSensorState)
{
    int16_t availableSamples = MAX30102_GetNumSamplesInFifo(pSensorState->pHI2C);

    if (pSensorState->started != 0 && availableSamples > 0)
    {
        debug_write_string("==> Z: "); debug_write_int(availableSamples); debug_write_newline();
    }

    uint8_t sample[6];
    uint32_t value;

    for (int16_t i = 0; i < availableSamples; ++i)
    {
        MAX30102_ReadFifo(pSensorState->pHI2C, sample, 6);

		  if (pSensorState->started != 0)
		  {
			  if (pSensorState->usingPpg)
			  {
		          value = ((sample[3] << 16) & 0x03) + (sample[4] << 8) + sample[5];
				  debug_write_string(" IR: "); debug_write_int(value); debug_write_newline();

			  }
			  else
			  {
				value = ++pSensorState->ramp;
    	        debug_write_string(" RAMP: "); debug_write_int(value); debug_write_newline();
			  }

	          // put the value to our circular buffer for transmitting via USB
            ring_buffer_add_sample(pSensorState->pRingBuf, value);
		  }
    }
}

void X20_TransmitSamples(ring_buffer *pRingBuf, usb_package *transmit_buffer, uint16_t required_samples)
{
    if( ! CDC_FreeToTransmit() )
    {
        return;
    }

    uint16_t ring_buffer_samples = ring_buffer_get_count(pRingBuf);
    if (ring_buffer_samples >= required_samples)
    {
        // copy samples from the ring buffer to the transmit buffer
        for( uint16_t i = 0; i < required_samples; ++i)
        {
            transmit_buffer->samples[i] = ring_buffer_remove_sample(pRingBuf);
	    }
	    transmit_buffer->package_number++;
	    transmit_buffer->ring_buffer_data_count = ring_buffer_samples;
	    transmit_buffer->num_samples = required_samples;
	    transmit_buffer->ring_buffer_overflows = pRingBuf->overflows;
	    pRingBuf->overflows = 0;

	    uint16_t len = sizeof(usb_package) + transmit_buffer->num_samples * sizeof(TR_BUF_SAMPLE_T);
	    // transfer the package to the USB Host
        int start_tr = HAL_GetTick();
        int result = CDC_Transmit_FS((uint8_t*)transmit_buffer, len);

        int stop_tr = HAL_GetTick();
        debug_write_string("TR_TIME: "); debug_write_int(stop_tr - start_tr); debug_write_newline();
    }
}

