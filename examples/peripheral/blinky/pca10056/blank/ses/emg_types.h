#ifndef EMG_TYPES_H
#define EMG_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t ch1;  // sign-extended 24-bit sample for channel 1
    int32_t ch2;  // sign-extended 24-bit sample for channel 2
} emg_sample_t;

#endif // EMG_TYPES_H
