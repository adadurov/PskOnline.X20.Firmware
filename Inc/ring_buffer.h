

typedef struct
{
  // write index
  uint16_t last;

  // read index
  uint16_t first;

  // buffer size
  uint16_t size;

  // buffer overflow count
  // will not exceed INT32_MAX
  int32_t overflows;

  // flexible buffer
  uint32_t buffer[];

} ring_buffer;

ring_buffer *ring_buffer_alloc(uint16_t size);
void ring_buffer_free(ring_buffer* buffer);

void ring_buffer_clear(ring_buffer* buffer);

void ring_buffer_add_sample(ring_buffer* buffer, uint32_t sample);

uint32_t ring_buffer_remove_sample(ring_buffer* buffer);

uint16_t ring_buffer_get_count(ring_buffer* buffer);
