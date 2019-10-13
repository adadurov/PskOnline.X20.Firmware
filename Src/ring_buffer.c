
#include "stm32f1xx.h"
#include "stdlib.h"

#include "ring_buffer.h"

ring_buffer *ring_buffer_alloc(uint16_t num_samples)
{
	ring_buffer *b = (ring_buffer*)malloc(sizeof(ring_buffer) + sizeof(uint32_t) * num_samples);
	b->size = num_samples;
	b->begin = 0;
	b->end = 0;
	b->overflows = 0;
	return b;
}
void ring_buffer_free(ring_buffer* buffer)
{
	free(buffer);
}

void ring_buffer_clear(ring_buffer* buffer)
{
	buffer->begin = 0;
	buffer->end = 0;
	buffer->overflows = 0;
}

uint16_t inc_and_wrap(ring_buffer *buffer, uint16_t ring_buf_index)
{
	++ring_buf_index;
	if (ring_buf_index > buffer->size - 1)
	{
		ring_buf_index = 0;
	}
	return ring_buf_index;
}

void ring_buffer_add_sample(ring_buffer* buffer, uint32_t sample)
{
	buffer->buffer[buffer->begin] = sample;
	uint16_t size_before = ring_buffer_get_count(buffer);
	buffer->begin = inc_and_wrap(buffer, buffer->begin);
	uint16_t size_after = ring_buffer_get_count(buffer);
	if( size_after < size_before )
	{
		buffer->overflows += 1;
	}
}

uint16_t ring_buffer_get_count(ring_buffer* buffer)
{
	if (buffer->end >= buffer->begin)
	{
		return buffer->end - buffer->begin;
	}
	return buffer->size - (buffer->begin - buffer->end);
}

uint32_t ring_buffer_remove_sample(ring_buffer* buffer)
{
	uint32_t value = buffer->buffer[buffer->end];
	buffer->end = inc_and_wrap(buffer, buffer->end);
	return value;
}


