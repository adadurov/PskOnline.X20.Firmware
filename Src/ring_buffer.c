
#include "stm32f1xx.h"
#include "stdlib.h"

#include "ring_buffer.h"

ring_buffer *ring_buffer_alloc(uint16_t num_samples)
{
	ring_buffer *b = (ring_buffer*)malloc(sizeof(ring_buffer) + sizeof(uint32_t) * num_samples);
	b->size = num_samples;
	b->last = 0;
	b->first = 0;
	b->overflows = 0;
	return b;
}
void ring_buffer_free(ring_buffer* buffer)
{
	free(buffer);
}

void ring_buffer_clear(ring_buffer* buffer)
{
	buffer->last = 0;
	buffer->first = 0;
	buffer->overflows = 0;
}

uint16_t inc_and_wrap(ring_buffer *buffer, uint16_t ring_buf_index)
{
	++ring_buf_index;
	if (ring_buf_index == buffer->size)
	{
		ring_buf_index = 0;
	}
	return ring_buf_index;
}

void ring_buffer_add_sample(ring_buffer* buffer, uint32_t sample)
{
	buffer->buffer[buffer->last] = sample;

	uint16_t size_before = ring_buffer_get_count(buffer);
	buffer->last = inc_and_wrap(buffer, buffer->last);
	uint16_t size_after = ring_buffer_get_count(buffer);
	if( size_after < size_before )
	{
		if (buffer->overflows < INT32_MAX)
		{
			buffer->overflows += 1;
		}
	}
}

uint16_t ring_buffer_get_count(ring_buffer* buffer)
{
	if (buffer->last >= buffer->first)
	{
		return buffer->last - buffer->first;
	}
	return buffer->size - (buffer->first - buffer->last);
}

uint32_t ring_buffer_remove_sample(ring_buffer* buffer)
{
	uint32_t value = buffer->buffer[buffer->first];
	buffer->first = inc_and_wrap(buffer, buffer->first);
	return value;
}


