#ifndef ADS1292R_H
#define ADS1292R_H

#include <stdint.h>
#include <stdbool.h>
#include "emg_types.h"

// ADS1292R opcodes (see TI datasheet)
#define ADS1292R_CMD_RESET   0x06
#define ADS1292R_CMD_START   0x08
#define ADS1292R_CMD_STOP    0x0A
#define ADS1292R_CMD_RDATAC  0x10
#define ADS1292R_CMD_SDATAC  0x11
#define ADS1292R_CMD_RREG    0x20
#define ADS1292R_CMD_WREG    0x40

// Register addresses (partial; enough for basic 2-ch EMG config)
#define ADS1292R_REG_ID       0x00
#define ADS1292R_REG_CONFIG1  0x01
#define ADS1292R_REG_CONFIG2  0x02
#define ADS1292R_REG_LOFF     0x03
#define ADS1292R_REG_CH1SET   0x04
#define ADS1292R_REG_CH2SET   0x05
#define ADS1292R_REG_RLD_SENS 0x06
#define ADS1292R_REG_LOFF_SENS 0x07
#define ADS1292R_REG_LOFF_STAT 0x08
#define ADS1292R_REG_RESP1    0x09
#define ADS1292R_REG_RESP2    0x0A

// Public API

// Initialise ADS1292R for 2 kSPS, internal 2.42 V reference, PGA = x6,
// both channels enabled for differential EMG.
bool ads1292r_init(void);

// Start continuous data conversion.
bool ads1292r_start_continuous(void);

// Stop continuous data conversion.
bool ads1292r_stop_continuous(void);

// Read one sample frame from the device (status + 2 channels of 24-bit data).
// This function assumes DRDY is already low when called.
bool ads1292r_read_sample(emg_sample_t *out_sample);

#endif // ADS1292R_H
