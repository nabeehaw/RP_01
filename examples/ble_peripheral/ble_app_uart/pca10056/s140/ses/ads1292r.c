#include "ads1292r.h"
#include "platform.h"

#define ADS1292R_FRAME_LEN  9u

/* ====================================================================
 * ADS1292R Register Configuration Values
 *
 * All values derived from the ADS1292R datasheet and the Kinetic Link
 * PCB schematic (Student A's design).
 * ==================================================================== */

/* CONFIG1 = 0x04
 *   bit 7    : 0  (reserved)
 *   bit 6    : 0  (DAISY_IN disabled)
 *   bit 5    : 0  (CLK_EN, oscillator clock output off)
 *   bits 4-3 : 00 (reserved)
 *   bits 2-0 : 100 = 2 kSPS data rate                                */
#define CFG_CONFIG1     0x04u

/* CONFIG2 = 0xA0
 *   bit 7    : 1  (must be 1 for ADS1292R)
 *   bit 6    : 0  (lead-off comparators off)
 *   bit 5    : 1  (PDB_REFBUF — internal reference buffer ON)
 *   bit 4    : 0  (VREF = 2.42 V, not 4.033 V)
 *   bit 3    : 0  (CLK_EN off)
 *   bit 2    : 0  (reserved)
 *   bit 1    : 0  (INT_TEST — no internal test signal)
 *   bit 0    : 0  (TEST_FREQ)                                        */
#define CFG_CONFIG2     0xA0u

/* CH1SET = 0x00  (PD off, Gain = 6, MUX = normal input)
 * CH2SET = 0x00  (same)                                               */
#define CFG_CH1SET      0x00u
#define CFG_CH2SET      0x00u

/* RLD_SENS = 0x13
 *   bit 5    : 0  (PGA chop = default)
 *   bit 4    : 1  (PDB_RLD — power on the RLD buffer amplifier)
 *   bit 3    : 0  (RLD_LOFF_SENS — RLD not used for lead-off)
 *   bit 2    : 0  (RLD2N — CH2 neg not routed to RLD)
 *   bit 1    : 1  (RLD2P — CH2 pos → RLD derivation)
 *   bit 0    : 1  (RLD1P — CH1 pos → RLD derivation)
 *
 * This enables common-mode rejection using the Right Leg Drive circuit
 * (1 MΩ + 1.5 nF feedback network on the PCB, ~100 Hz loop BW).
 * Without this register write, the RLD amplifier is powered down and
 * mains interference is NOT actively cancelled — leading to large
 * 60 Hz artifacts and odd RMS readings.                               */
#define CFG_RLD_SENS    0x13u

/* LOFF = 0x00 (lead-off detection disabled for now)                   */
#define CFG_LOFF        0x00u

/* LOFF_SENS = 0x00 (all lead-off channels disconnected)               */
#define CFG_LOFF_SENS   0x00u

/* ====================================================================
 * Helpers
 * ==================================================================== */

static void ads1292r_cmd_delay(void)
{
    platform_delay_ms(1u);
}

static int32_t sign_extend_24(int32_t x)
{
    if ((x & 0x00800000) != 0)
    {
        x |= (int32_t)0xFF000000;
    }
    return x;
}

static void ads1292r_send_command(uint8_t cmd)
{
    uint8_t tx[1] = { cmd };

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, NULL, 1u);
    platform_ads1292r_cs_high();

    ads1292r_cmd_delay();
}

/* ====================================================================
 * Register read / write
 * ==================================================================== */

bool ads1292r_read_reg(uint8_t addr, uint8_t *value_out)
{
    uint8_t tx[3];
    uint8_t rx[3];

    if (value_out == NULL)
    {
        return false;
    }

    tx[0] = (uint8_t)(ADS1292R_CMD_RREG | (addr & 0x1Fu));
    tx[1] = 0x00u;   /* read 1 register */
    tx[2] = 0x00u;   /* dummy byte to clock data out */

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, rx, 3u);
    platform_ads1292r_cs_high();

    ads1292r_cmd_delay();

    *value_out = rx[2];
    return true;
}

bool ads1292r_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t tx[3];

    tx[0] = (uint8_t)(ADS1292R_CMD_WREG | (addr & 0x1Fu));
    tx[1] = 0x00u;   /* write 1 register */
    tx[2] = value;

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, NULL, 3u);
    platform_ads1292r_cs_high();

    ads1292r_cmd_delay();

    return true;
}

bool ads1292r_read_id(uint8_t *id_out)
{
    if (id_out == NULL)
    {
        return false;
    }
    return ads1292r_read_reg(ADS1292R_REG_ID, id_out);
}

/* ====================================================================
 * Read-back verification helper
 *
 * Writes a register and reads it back to confirm the write took.
 * Returns true if the read-back matches the written value.
 * ==================================================================== */
static bool ads1292r_write_verify(uint8_t addr, uint8_t value,
                                  const char *name)
{
    if (!ads1292r_write_reg(addr, value))
    {
        platform_log("ADS: WREG %s failed\r\n", name);
        return false;
    }

    uint8_t readback = 0;
    if (!ads1292r_read_reg(addr, &readback))
    {
        platform_log("ADS: RREG %s failed\r\n", name);
        return false;
    }

    if (readback != value)
    {
        platform_log("ADS: %s wrote 0x%02X read 0x%02X MISMATCH\r\n",
                     name, value, readback);
        return false;
    }

    platform_log("ADS: %s = 0x%02X OK\r\n", name, readback);
    return true;
}

/* ====================================================================
 * Initialisation
 * ==================================================================== */

bool ads1292r_init(void)
{
    uint8_t id = 0u;
    uint8_t tries;

    platform_log("ADS1292R init...\r\n");

    /* Hardware reset via SPI command */
    ads1292r_send_command(ADS1292R_CMD_RESET);
    platform_delay_ms(10u);

    /* Stop any continuous read that may be in progress */
    ads1292r_send_command(ADS1292R_CMD_SDATAC);
    platform_delay_ms(2u);

    /* Read device ID (expect 0x53 for ADS1292R, 0x73 for ADS1292) */
    for (tries = 0; tries < 5; tries++)
    {
        if (!ads1292r_read_id(&id))
        {
            continue;
        }

        platform_log("ADS1292R ID try %u = 0x%02X\r\n", tries + 1u, id);

        if ((id != 0x00u) && (id != 0xFFu))
        {
            break;
        }

        platform_delay_ms(2u);
    }

    if ((id == 0x00u) || (id == 0xFFu))
    {
        platform_log("ADS1292R: invalid ID — check SPI wiring\r\n");
        return false;
    }

    /* Write configuration registers with read-back verification */
    bool ok = true;

    ok = ok && ads1292r_write_verify(ADS1292R_REG_CONFIG1,  CFG_CONFIG1,
                                     "CONFIG1");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_CONFIG2,  CFG_CONFIG2,
                                     "CONFIG2");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_LOFF,     CFG_LOFF,
                                     "LOFF");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_CH1SET,   CFG_CH1SET,
                                     "CH1SET");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_CH2SET,   CFG_CH2SET,
                                     "CH2SET");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_RLD_SENS, CFG_RLD_SENS,
                                     "RLD_SENS");
    ok = ok && ads1292r_write_verify(ADS1292R_REG_LOFF_SENS, CFG_LOFF_SENS,
                                     "LOFF_SENS");

    if (!ok)
    {
        platform_log("ADS1292R: register config verification FAILED\r\n");
        /* Continue anyway — some registers may not read back identically
         * on all silicon revisions.  The data will reveal if it worked. */
    }

    platform_delay_ms(10u);
    platform_log("ADS1292R init done (ID=0x%02X)\r\n", id);

    return true;
}

/* ====================================================================
 * Continuous read mode
 * ==================================================================== */

bool ads1292r_start_continuous(void)
{
    ads1292r_send_command(ADS1292R_CMD_START);
    ads1292r_send_command(ADS1292R_CMD_RDATAC);
    return true;
}

bool ads1292r_stop_continuous(void)
{
    ads1292r_send_command(ADS1292R_CMD_SDATAC);
    ads1292r_send_command(ADS1292R_CMD_STOP);
    return true;
}

/* ====================================================================
 * Read one 9-byte sample frame
 *
 * Frame layout (MSB first):
 *   [0..2]  Status (24 bits — lead-off, GPIO, etc.)
 *   [3..5]  CH1 data (24-bit signed)
 *   [6..8]  CH2 data (24-bit signed)
 *
 * Caller must ensure DRDY is low before calling.
 * ==================================================================== */

bool ads1292r_read_sample(emg_sample_t *out_sample)
{
    uint8_t tx[ADS1292R_FRAME_LEN] = {0};
    uint8_t rx[ADS1292R_FRAME_LEN] = {0};

    if (out_sample == NULL)
    {
        return false;
    }

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, rx, ADS1292R_FRAME_LEN);
    platform_ads1292r_cs_high();

    /* Assemble 24-bit codes from 3 bytes each */
    int32_t ch1 = ((int32_t)rx[3] << 16)
                | ((int32_t)rx[4] << 8)
                | ((int32_t)rx[5]);

    int32_t ch2 = ((int32_t)rx[6] << 16)
                | ((int32_t)rx[7] << 8)
                | ((int32_t)rx[8]);

    out_sample->ch1 = sign_extend_24(ch1);
    out_sample->ch2 = sign_extend_24(ch2);

    return true;
}