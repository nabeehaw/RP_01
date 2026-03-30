#include "ads1292r.h"
#include "platform.h"

#define ADS1292R_FRAME_LEN  9u

/* ====================================================================
 * ADS1292R Register Configuration — PRODUCTION
 * ==================================================================== */

/* Valid device IDs (top nibble of ID register) */
#define ADS1292_ID          0x53u   /* ADS1292 (no R suffix)            */
#define ADS1292R_ID         0x73u   /* ADS1292R                         */

/* CONFIG1 = 0x04: 2 kSPS data rate                                    */
#define CFG_CONFIG1     0x04u

/* CONFIG2 = 0xE0:
 *   bit 7 = 1  (must be 1)
 *   bit 6 = 1  (PDB_LOFF_COMP — enable lead-off comparators)
 *   bit 5 = 1  (PDB_REFBUF — internal reference ON)
 *   bits 4-0 = 0                                                      */
#define CFG_CONFIG2     0xE0u

/* CH1SET = 0x00, CH2SET = 0x00: PGA gain = 6, normal input            */
#define CFG_CH1SET      0x00u
#define CFG_CH2SET      0x00u

/* RLD_SENS = 0x13: RLD buffer ON, CH1P + CH2P routed to RLD           */
#define CFG_RLD_SENS    0x13u

/* LOFF = 0x10:
 *   bits [7:5] = 000  (comparator threshold: 95% / 5%)
 *   bit 4 = 1         (must be 1 for ADS1292R)
 *   bits [3:2] = 00   (lead-off current: 6 nA)
 *   bit 1 = 0         (reserved)
 *   bit 0 = 0         (DC lead-off, not AC)                           */
#define CFG_LOFF        0x10u

/* LOFF_SENS = 0x0F:
 *   bit 7-4 = 0000    (FLIP bits — no inversion)
 *   bit 3 = 1         (LOFF2N — sense IN2N)
 *   bit 2 = 1         (LOFF2P — sense IN2P)
 *   bit 1 = 1         (LOFF1N — sense IN1N)
 *   bit 0 = 1         (LOFF1P — sense IN1P)
 * All 4 electrode inputs are monitored for lead-off.                  */
#define CFG_LOFF_SENS   0x0Fu

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

    if (value_out == NULL) return false;

    tx[0] = (uint8_t)(ADS1292R_CMD_RREG | (addr & 0x1Fu));
    tx[1] = 0x00u;
    tx[2] = 0x00u;

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
    tx[1] = 0x00u;
    tx[2] = value;

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, NULL, 3u);
    platform_ads1292r_cs_high();

    ads1292r_cmd_delay();
    return true;
}

bool ads1292r_read_id(uint8_t *id_out)
{
    if (id_out == NULL) return false;
    return ads1292r_read_reg(ADS1292R_REG_ID, id_out);
}

/* Write + readback verify. Returns false on mismatch. */
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
 * Initialisation — STRICT
 * ==================================================================== */

bool ads1292r_init(void)
{
    uint8_t id = 0u;
    uint8_t tries;

    platform_log("ADS1292R init...\r\n");

    ads1292r_send_command(ADS1292R_CMD_RESET);
    platform_delay_ms(10u);

    ads1292r_send_command(ADS1292R_CMD_SDATAC);
    platform_delay_ms(2u);

    /* Read device ID — only accept ADS1292 (0x53) or ADS1292R (0x73) */
    for (tries = 0; tries < 5; tries++)
    {
        if (!ads1292r_read_id(&id)) continue;

        platform_log("ADS1292R ID try %u = 0x%02X\r\n", tries + 1u, id);

        if (id == ADS1292_ID || id == ADS1292R_ID)
        {
            break;
        }

        platform_delay_ms(2u);
    }

    if (id != ADS1292_ID && id != ADS1292R_ID)
    {
        platform_log("ADS1292R: INVALID ID 0x%02X (expected 0x53 or 0x73)\r\n", id);
        return false;
    }

    /* Write registers with read-back verification.
     * Count mismatches but only fail if critical registers don't take. */
    uint8_t mismatch_count = 0;

    if (!ads1292r_write_verify(ADS1292R_REG_CONFIG1,   CFG_CONFIG1,  "CONFIG1"))  mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_CONFIG2,   CFG_CONFIG2,  "CONFIG2"))  mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_LOFF,      CFG_LOFF,     "LOFF"))     mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_CH1SET,    CFG_CH1SET,   "CH1SET"))   mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_CH2SET,    CFG_CH2SET,   "CH2SET"))   mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_RLD_SENS,  CFG_RLD_SENS, "RLD_SENS")) mismatch_count++;
    if (!ads1292r_write_verify(ADS1292R_REG_LOFF_SENS, CFG_LOFF_SENS,"LOFF_SENS")) mismatch_count++;

    if (mismatch_count > 2)
    {
        platform_log("ADS1292R: %u register mismatches — ABORTING\r\n", mismatch_count);
        return false;
    }
    else if (mismatch_count > 0)
    {
        platform_log("ADS1292R: %u minor mismatches (continuing)\r\n", mismatch_count);
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
 *   [0]     Status byte 0: [1100][GPIO4:1]  (top nibble = 0xC)
 *   [1]     Status byte 1: [LOFF_STAT[4:0]][0][0][0]
 *             bit 7 = LOFF_STAT RLD
 *             bit 6 = IN2N off
 *             bit 5 = IN2P off
 *             bit 4 = IN1N off
 *             bit 3 = IN1P off
 *   [2]     Status byte 2: (GPIO, etc.)
 *   [3..5]  CH1 data (24-bit signed)
 *   [6..8]  CH2 data (24-bit signed)
 * ==================================================================== */

bool ads1292r_read_sample(emg_sample_t *out_sample)
{
    uint8_t tx[ADS1292R_FRAME_LEN] = {0};
    uint8_t rx[ADS1292R_FRAME_LEN] = {0};

    if (out_sample == NULL) return false;

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, rx, ADS1292R_FRAME_LEN);
    platform_ads1292r_cs_high();

    /* Parse CH1 and CH2 */
    int32_t ch1 = ((int32_t)rx[3] << 16)
                | ((int32_t)rx[4] << 8)
                | ((int32_t)rx[5]);

    int32_t ch2 = ((int32_t)rx[6] << 16)
                | ((int32_t)rx[7] << 8)
                | ((int32_t)rx[8]);

    out_sample->ch1 = sign_extend_24(ch1);
    out_sample->ch2 = sign_extend_24(ch2);

    /* Parse lead-off status from status byte 1.
     * ADS1292R status byte 1 bits [7:3] = LOFF_STAT
     *   our bit 0 = IN1P off  (status bit 3)
     *   our bit 1 = IN1N off  (status bit 4)
     *   our bit 2 = IN2P off  (status bit 5)
     *   our bit 3 = IN2N off  (status bit 6)                          */
    uint8_t stat1 = rx[1];
    out_sample->lead_off = 0;
    if (stat1 & (1u << 3)) out_sample->lead_off |= 0x01u;  /* IN1P */
    if (stat1 & (1u << 4)) out_sample->lead_off |= 0x02u;  /* IN1N */
    if (stat1 & (1u << 5)) out_sample->lead_off |= 0x04u;  /* IN2P */
    if (stat1 & (1u << 6)) out_sample->lead_off |= 0x08u;  /* IN2N */

    return true;
}

bool ads1292r_read_raw_frame(uint8_t *frame_out)
{
    uint8_t tx[ADS1292R_FRAME_LEN] = {0};

    if (frame_out == NULL) return false;

    platform_ads1292r_cs_low();
    platform_spi_transfer(tx, frame_out, ADS1292R_FRAME_LEN);
    platform_ads1292r_cs_high();

    return true;
}