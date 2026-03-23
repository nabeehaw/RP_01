#ifndef EMG_BUFFER_H
#define EMG_BUFFER_H

#include <stddef.h>
#include <stdbool.h>
#include "emg_types.h"

// Simple ring buffer for EMG samples

#ifndef EMG_BUFFER_CAPACITY
#define EMG_BUFFER_CAPACITY 2048u
#endif

typedef struct
{
    emg_sample_t data[EMG_BUFFER_CAPACITY];
    size_t head;   // index of next write
    size_t tail;   // index of next read
    size_t count;  // number of elements currently stored
} emg_buffer_t;

void emg_buffer_init(emg_buffer_t *buf);
bool emg_buffer_push(emg_buffer_t *buf, const emg_sample_t *sample);
bool emg_buffer_pop(emg_buffer_t *buf, emg_sample_t *out_sample);
size_t emg_buffer_count(const emg_buffer_t *buf);

#endif // EMG_BUFFER_H
