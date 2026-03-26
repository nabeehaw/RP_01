#include "platform.h"

#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrfx_spim.h"
#include "SEGGER_RTT.h"
#include "app_error.h"

// ============================
// Pin definitions 
// ============================

// Using Flux schematic
#define PIN_DEBUG_LED       8    // P0.08 for debug LED

#define PIN_ADS1292R_DRDY   11   // P0.11 for DRDY (data ready)
#define PIN_ADS1292R_CS     12   // P0.12 for chip select

#define PIN_SPI_SCK         13   // P0.13 (AFE SCLK)
#define PIN_SPI_MISO        14   // P0.14
#define PIN_SPI_MOSI        15   // P0.15

#define PIN_ADS1292R_RESET  16   // P0.16  AFE_RESET
#define PIN_ADS1292R_START  17   // P0.17  AFE_START

#define PIN_UART_TX         20   // P0.20  UART_TX (via R5)
#define PIN_UART_RX         21   // P0.21  UART_RX (via R12)


// Max transfer length we ever use (we only need 9 bytes for ADS1292R read)
#define SPI_MAX_XFER_LEN    32

static const nrfx_spim_t m_spim = {
    .p_reg        = NRF_SPIM0,
    .drv_inst_idx = 0,   // we don’t care about the NRFX_SPIM0_INST_IDX macro
};

// ============================
// Platform implementation
// ============================

void platform_init(void)
{
    // RTT init (optional; SEGGER_RTT works even without explicit init)
    SEGGER_RTT_Init();

    // Configure LED
    nrf_gpio_cfg_output(PIN_DEBUG_LED);
    nrf_gpio_pin_clear(PIN_DEBUG_LED);  // LED off

    // Wake up the ADS1292R (Pull RESET High)
    nrf_gpio_cfg_output(PIN_ADS1292R_RESET);
    nrf_gpio_pin_set(PIN_ADS1292R_RESET);

    // Keep START Low so we can control conversions via SPI
    nrf_gpio_cfg_output(PIN_ADS1292R_START);
    nrf_gpio_pin_clear(PIN_ADS1292R_START);

    // Configure ADS1292R CS as output, idle high
    nrf_gpio_cfg_output(PIN_ADS1292R_CS);
    nrf_gpio_pin_set(PIN_ADS1292R_CS);

    // Configure ADS1292R DRDY as input (no pull; board should provide it)
    nrf_gpio_cfg_input(PIN_ADS1292R_DRDY, NRF_GPIO_PIN_NOPULL);

    // Configure SPI (SPIM)
    nrfx_spim_config_t spi_cfg = NRFX_SPIM_DEFAULT_CONFIG;

    spi_cfg.sck_pin      = PIN_SPI_SCK;
    spi_cfg.mosi_pin     = PIN_SPI_MOSI;
    spi_cfg.miso_pin     = PIN_SPI_MISO;
    spi_cfg.ss_pin       = NRFX_SPIM_PIN_NOT_USED; // we drive CS manually
    spi_cfg.frequency    = NRF_SPIM_FREQ_250K;   // 250K is plenty ADS1292R
    spi_cfg.mode         = NRF_SPIM_MODE_1;        // CPOL = 0, CPHA = 1 per datasheet
    spi_cfg.bit_order    = NRF_SPIM_BIT_ORDER_MSB_FIRST;

    ret_code_t err = nrfx_spim_init(&m_spim, &spi_cfg, NULL, NULL);
    APP_ERROR_CHECK(err);

    platform_log("platform_init: SPI + GPIO + RTT initialised\n");
}

void platform_delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

void platform_log(const char *fmt, ...)
{
    char buffer[128];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    SEGGER_RTT_WriteString(0, buffer);
}

void platform_led_set(bool on)
{
    if (on)
    {
        nrf_gpio_pin_set(PIN_DEBUG_LED);
    }
    else
    {
        nrf_gpio_pin_clear(PIN_DEBUG_LED);
    }
}

void platform_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (len == 0u)
    {
        return;
    }

    if (len > SPI_MAX_XFER_LEN)
    {
        // Hard fail – you misconfigured something
        APP_ERROR_HANDLER(len);
    }

    uint8_t dummy_tx[SPI_MAX_XFER_LEN] = { 0 };
    uint8_t dummy_rx[SPI_MAX_XFER_LEN] = { 0 };

    const uint8_t *tx_buf = tx ? tx : dummy_tx;
    uint8_t       *rx_buf = rx ? rx : dummy_rx;

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length   = (size_t)len,
        .p_rx_buffer = rx_buf,
        .rx_length   = (size_t)len
    };

    ret_code_t err = nrfx_spim_xfer(&m_spim, &xfer, 0);
    APP_ERROR_CHECK(err);

    // If caller provided an rx buffer, the data is already in rx_buf (== rx).
    // If caller didn't provide rx, we discard dummy_rx.
}

void platform_ads1292r_cs_low(void)
{
    nrf_gpio_pin_clear(PIN_ADS1292R_CS);
}

void platform_ads1292r_cs_high(void)
{
    nrf_gpio_pin_set(PIN_ADS1292R_CS);
}

bool platform_ads1292r_drdy_is_low(void)
{
    // DRDY is active low. Return true when a new conversion is ready.
    return (nrf_gpio_pin_read(PIN_ADS1292R_DRDY) == 0u);
}

void platform_ads1292r_reset_low(void)
{
    nrf_gpio_pin_clear(PIN_ADS1292R_RESET);
}

void platform_ads1292r_reset_high(void)
{
    nrf_gpio_pin_set(PIN_ADS1292R_RESET);
}

void platform_ads1292r_start_low(void)
{
    nrf_gpio_pin_clear(PIN_ADS1292R_START);
}

void platform_ads1292r_start_high(void)
{
    nrf_gpio_pin_set(PIN_ADS1292R_START);
}