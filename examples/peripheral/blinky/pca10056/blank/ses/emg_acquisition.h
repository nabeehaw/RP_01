#ifndef EMG_ACQUISITION_H
#define EMG_ACQUISITION_H

#include <stdbool.h>
#include "emg_types.h"
#include "emg_buffer.h"

// Global EMG buffer instance (defined in emg_acquisition.c)
extern emg_buffer_t g_emg_buffer;

// Initialise EMG acquisition: buffer + ADS1292R config.
bool emg_acquisition_init(void);

// Polling function to be called frequently from the main loop.
// When DRDY is low, read a new sample and push it into the buffer.
void emg_acquisition_poll(void);

#endif // EMG_ACQUISITION_H
