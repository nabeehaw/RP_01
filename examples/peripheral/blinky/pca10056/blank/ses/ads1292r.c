#include "ads1292r.h"
#include "platform.h"

// Helper: send single-byte command
static void ads1292r_send_command(uint8_t cmd)
{
    platform_ads1292r_cs_low();
    uint8_t tx[1] = { cmd };
    platform_spi_transfer(tx, NULL, 1u);
    platform_ads1292r_cs_high();
}

// Helper: write register
static void ads1292r_write_reg(uint8_t addr, uint8_t value)
{
    platform_ads1292r_cs_low();

    uint8_t tx[3];
    tx[0] = ADS1292R_CMD_WREG | (addr & 0x1F);  // WREG + addr
    tx[1] = 0x00;                               // write 1 register
    tx[2] = value;
    platform_spi_transfer(tx, NULL, 3u);

    platform_ads1292r_cs_high();
}

// Helper: read register
static uint8_t ads1292r_read_reg(uint8_t addr)
{
    platform_ads1292r_cs_low();

    uint8_t tx[3];
    uint8_t rx[3];
    tx[0] = ADS1292R_CMD_RREG | (addr & 0x1F);  // RREG + addr
    tx[1] = 0x00;                               // read 1 register
    tx[2] = 0x00;                               // dummy
    platform_spi_transfer(tx, rx, 3u);

    platform_ads1292r_cs_high();

    return rx[2];
}

static int32_t sign_extend_24(int32_t x)
{
    // x is 24-bit two's complement in bits [23:0]
    if (x & 0x00800000)
    {
        x |= 0xFF000000; // set upper bits if sign bit set
    }
    return x;
}

bool ads1292r_init(void)
{
    platform_log("ADS1292R init...\n");

    // Reset the device
    ads1292r_send_command(ADS1292R_CMD_RESET);
    platform_delay_ms(10u);

    // Issue SDATAC to stop any conversion before config
    ads1292r_send_command(ADS1292R_CMD_SDATAC);

    // Read ID to sanity-check SPI communication
    uint8_t id = ads1292r_read_reg(ADS1292R_REG_ID);
    platform_log("ADS1292R ID = 0x%02X\n", (unsigned)id);

    // CONFIG1: continuous conversion, 2-kSPS data rate.
    // Bits: SINGLE_SHOT=0, [6:3]=0, DR[2:0]=100b => 0x04.
    uint8_t config1 = 0x04u;
    ads1292r_write_reg(ADS1292R_REG_CONFIG1, config1);

    // CONFIG2: enable internal reference buffer, 2.42-V reference,
    // internal oscillator, test signals off.
    // Bits: [7]=1 (must be 1), PDB_LOFF_COMP=0, PDB_REFBUF=1,
    // VREF_4V=0 (2.42 V), CLK_EN=0 (no clock output), INT_TEST=0, TEST_FREQ=0.
    uint8_t config2 = 0xA0u;  // 1010 0000b
    ads1292r_write_reg(ADS1292R_REG_CONFIG2, config2);

    // CH1SET / CH2SET: leave default gain = 6 and normal electrode inputs.
    // Reset value is 0x00 => PDx=0, GAINx[2:0]=000 (gain=6), MUXx[3:0]=0000.
    ads1292r_write_reg(ADS1292R_REG_CH1SET, 0x00u);
    ads1292r_write_reg(ADS1292R_REG_CH2SET, 0x00u);

    platform_delay_ms(10u);

    return true;
}

bool ads1292r_start_continuous(void)
{
    // Start conversions and enable RDATAC mode
    ads1292r_send_command(ADS1292R_CMD_START);
    platform_delay_ms(1u);
    ads1292r_send_command(ADS1292R_CMD_RDATAC);
    platform_delay_ms(1u);
    return true;
}

bool ads1292r_stop_continuous(void)
{
    // Stop continuous read and conversions
    ads1292r_send_command(ADS1292R_CMD_SDATAC);
    platform_delay_ms(1u);
    ads1292r_send_command(ADS1292R_CMD_STOP);
    platform_delay_ms(1u);
    return true;
}

bool ads1292r_read_sample(emg_sample_t *out_sample)
{
    if (out_sample == NULL)
    {
        return false;
    }

    // Caller should ensure DRDY is low before calling this.
    // Each frame: 3 status bytes + 3 bytes per channel (2 channels) = 9 bytes total.

    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, rx, 9u);
    platform_ads1292r_cs_high();

    // Combine bytes into 24-bit signed values
    int32_t ch1 = ((int32_t)rx[3] << 16) | ((int32_t)rx[4] << 8) | (int32_t)rx[5];
    int32_t ch2 = ((int32_t)rx[6] << 16) | ((int32_t)rx[7] << 8) | (int32_t)rx[8];

    ch1 = sign_extend_24(ch1);
    ch2 = sign_extend_24(ch2);

    out_sample->ch1 = ch1;
    out_sample->ch2 = ch2;

    return true;
}
