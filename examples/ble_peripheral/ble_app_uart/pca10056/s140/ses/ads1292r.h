#ifndef ADS1292R_H
#define ADS1292R_H

#include <stdint.h>
#include <stdbool.h>
#include "emg_types.h"

// ============================
// ADS1292R opcodes
// ============================
#define ADS1292R_CMD_WAKEUP   0x02
#define ADS1292R_CMD_STANDBY  0x04
#define ADS1292R_CMD_RESET    0x06
#define ADS1292R_CMD_START    0x08
#define ADS1292R_CMD_STOP     0x0A
#define ADS1292R_CMD_RDATAC   0x10
#define ADS1292R_CMD_SDATAC   0x11
#define ADS1292R_CMD_RDATA    0x12
#define ADS1292R_CMD_RREG     0x20
#define ADS1292R_CMD_WREG     0x40

// ============================
// Register addresses
// ============================
#define ADS1292R_REG_ID         0x00
#define ADS1292R_REG_CONFIG1    0x01
#define ADS1292R_REG_CONFIG2    0x02
#define ADS1292R_REG_LOFF       0x03
#define ADS1292R_REG_CH1SET     0x04
#define ADS1292R_REG_CH2SET     0x05
#define ADS1292R_REG_RLD_SENS   0x06
#define ADS1292R_REG_LOFF_SENS  0x07
#define ADS1292R_REG_LOFF_STAT  0x08
#define ADS1292R_REG_RESP1      0x09
#define ADS1292R_REG_RESP2      0x0A

// ============================
// Public API
// ============================

// Configure ADS1292R for basic 2-channel EMG operation.
// Returns false if SPI comms or ID read fail.
bool ads1292r_init(void);

// Read the device ID register.
// Returns true if SPI transaction succeeded.
bool ads1292r_read_id(uint8_t *id_out);

// Read one register.
bool ads1292r_read_reg(uint8_t addr, uint8_t *value_out);

// Write one register.
bool ads1292r_write_reg(uint8_t addr, uint8_t value);

// Start conversions and continuous read mode.
bool ads1292r_start_continuous(void);

// Stop continuous read mode and conversions.
bool ads1292r_stop_continuous(void);

// Read one 9-byte sample frame (status + CH1 + CH2).
// Assumes DRDY is already low when called.
bool ads1292r_read_sample(emg_sample_t *out_sample);

// Read one raw 9-byte SPI frame WITHOUT parsing.
// frame_out must point to a 9-byte buffer.
// Returns the raw bytes exactly as clocked from the ADS1292R.
bool ads1292r_read_raw_frame(uint8_t *frame_out);

#endif // ADS1292R_H