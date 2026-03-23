#include "emg_buffer.h"

void emg_buffer_init(emg_buffer_t *buf)
{
    buf->head = 0u;
    buf->tail = 0u;
    buf->count = 0u;
}

bool emg_buffer_push(emg_buffer_t *buf, const emg_sample_t *sample)
{
    if (buf->count >= EMG_BUFFER_CAPACITY)
    {
        // buffer full
        return false;
    }

    buf->data[buf->head] = *sample;
    buf->head = (buf->head + 1u) % EMG_BUFFER_CAPACITY;
    buf->count++;
    return true;
}

bool emg_buffer_pop(emg_buffer_t *buf, emg_sample_t *out_sample)
{
    if (buf->count == 0u)
    {
        return false;
    }

    *out_sample = buf->data[buf->tail];
    buf->tail = (buf->tail + 1u) % EMG_BUFFER_CAPACITY;
    buf->count--;
    return true;
}

size_t emg_buffer_count(const emg_buffer_t *buf)
{
    return buf->count;
}
