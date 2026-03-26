#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// This header defines a minimal hardware abstraction layer that you
// MUST implement using your MCU/SDK (nRF52, etc.).

// Initialise clocks, GPIO, SPI, UART/RTT, and any LEDs used for debug.
void platform_init(void);

// Millisecond delay (blocking is fine for bring-up).
void platform_delay_ms(uint32_t ms);

// Basic logging function (UART, RTT, etc.).
void platform_log(const char *fmt, ...);

// LED control for a single debug LED.
void platform_led_set(bool on);

// SPI transfer for ADS1292R.
// tx and rx can be NULL if you only need to read or write.
// Must assert CS before transfer and deassert after, or provide
// separate helpers for chip select.
void platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len);

void platform_ads1292r_reset_low(void);
void platform_ads1292r_reset_high(void);
void platform_ads1292r_start_low(void);
void platform_ads1292r_start_high(void);

// Control ADS1292R chip-select line.
void platform_ads1292r_cs_low(void);
void platform_ads1292r_cs_high(void);

// Read ADS1292R DRDY pin (true when low / data ready, or however you prefer).
bool platform_ads1292r_drdy_is_low(void);

#endif // PLATFORM_H
