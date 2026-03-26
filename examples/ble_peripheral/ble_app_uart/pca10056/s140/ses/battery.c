#include "battery.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_error.h"

/* ====================================================================
 * Pin and threshold definitions
 * ==================================================================== */

/* MCP73831 charger STAT net is connected to nRF P0.06.
 * Active-low:  0 = charging,  1 = not charging / standby.
 * R47 (10 kOhm) provides hardware pull-up to SYS_3V3.              */
#define CHG_STAT_PIN        NRF_GPIO_PIN_MAP(0, 6)

/* ---------- Post-LDO VDD thresholds (millivolts) ----------
 *
 * The nRF52840 is powered by a 3.3 V TLV700 LDO from the Li-ion cell.
 * While the battery is above ~3.5 V the LDO regulates and VDD = 3300 mV.
 * As the cell discharges below ~3.5 V the LDO enters dropout and VDD
 * begins to sag.
 *
 *  OK  :  VDD >= BATT_RECOVER_MV  (3200 mV)
 *  LOW :  VDD <  BATT_LOW_MV      (3100 mV)
 *
 * The 100 mV gap between LOW and RECOVER prevents toggling near the
 * transition point (hysteresis).                                      */
#define BATT_LOW_MV         3100u   /* Enter LOW when VDD drops below  */
#define BATT_RECOVER_MV     3200u   /* Exit LOW  when VDD rises above  */

/* Number of SAADC samples to average for noise reduction.             */
#define BATT_ADC_AVG_COUNT  4u

/* Flag-byte bit positions (must match emg_packet.h defines)           */
#define FLAG_BATT_OK        (1u << 0)   /* 0x01 */
#define FLAG_BATT_LOW       (1u << 1)   /* 0x02 */
#define FLAG_CHARGING       (1u << 2)   /* 0x04 */
/* bit 7 (0x80) = FAULT — set externally in main.c, not here          */

/* ====================================================================
 * Module state
 * ==================================================================== */
static batt_state_t m_batt_state   = BATT_STATE_UNKNOWN;
static uint16_t     m_cached_mv    = 0;
static bool         m_saadc_ready  = false;

/* ====================================================================
 * SAADC callback (required by driver, unused in blocking mode)
 * ==================================================================== */
static void saadc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    (void)p_event;
}

/* ====================================================================
 * Public API
 * ==================================================================== */

void battery_init(void)
{
    ret_code_t err;

    /* 1. Initialise SAADC driver (blocking / single-shot mode) */
    err = nrf_drv_saadc_init(NULL, saadc_callback);
    if (err == NRF_SUCCESS)
    {
        /* 2. Channel 0 → internal VDD (VDD/4 with 1/6 gain, 0.6 V ref) */
        nrf_saadc_channel_config_t ch_cfg =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

        err = nrf_drv_saadc_channel_init(0, &ch_cfg);
        m_saadc_ready = (err == NRF_SUCCESS);
    }

    /* 3. Configure CHG_STAT pin — NOPULL (R47 provides external pull-up) */
    nrf_gpio_cfg_input(CHG_STAT_PIN, NRF_GPIO_PIN_NOPULL);

    /* 4. Take an initial reading so cached state is valid immediately */
    if (m_saadc_ready)
    {
        m_cached_mv  = battery_read_mv();
        m_batt_state = battery_classify(m_cached_mv);
    }
}

/**
 * Read VDD via SAADC, averaged over BATT_ADC_AVG_COUNT samples.
 *
 * nRF52840 SAADC with NRF_SAADC_INPUT_VDD:
 *   VDD/4 is sampled with gain 1/6 against the 0.6 V internal reference.
 *   Full-scale = 0.6 V × 6 = 3.6 V   →   3600 mV / 1024 counts ≈ 3.515 mV/LSB.
 *   VDD_mV = (adc_count × 3600) / 1024
 */
uint16_t battery_read_mv(void)
{
    if (!m_saadc_ready)
    {
        return 0;
    }

    uint32_t sum = 0;

    for (uint8_t i = 0; i < BATT_ADC_AVG_COUNT; i++)
    {
        nrf_saadc_value_t adc_val = 0;
        ret_code_t err = nrf_drv_saadc_sample_convert(0, &adc_val);

        if (err != NRF_SUCCESS || adc_val < 0)
        {
            /* Treat failed / negative sample as 0 */
            continue;
        }
        sum += (uint32_t)adc_val;
    }

    uint32_t avg = sum / BATT_ADC_AVG_COUNT;
    uint32_t vdd_mv = (avg * 3600u) / 1024u;

    return (uint16_t)vdd_mv;
}

/**
 * Refresh the cached VDD reading and battery state.
 * Call from the main loop at ~4 Hz (every EMG tick).
 *
 * State machine with hysteresis:
 *   UNKNOWN / OK  →  LOW      when VDD < BATT_LOW_MV
 *   LOW           →  OK       when VDD >= BATT_RECOVER_MV
 *   (ADC read = 0 → stays in current state to avoid false LOW)
 */
void battery_update(void)
{
    uint16_t mv = battery_read_mv();

    /* Ignore failed reads (keep previous state) */
    if (mv == 0)
    {
        return;
    }

    m_cached_mv = mv;

    switch (m_batt_state)
    {
        case BATT_STATE_UNKNOWN:
        case BATT_STATE_OK:
            if (mv < BATT_LOW_MV)
            {
                m_batt_state = BATT_STATE_LOW;
            }
            else
            {
                m_batt_state = BATT_STATE_OK;
            }
            break;

        case BATT_STATE_LOW:
            if (mv >= BATT_RECOVER_MV)
            {
                m_batt_state = BATT_STATE_OK;
            }
            /* else: stay LOW */
            break;

        default:
            m_batt_state = BATT_STATE_UNKNOWN;
            break;
    }
}

/* ---- Simple queries ---- */

batt_state_t battery_get_state(void)
{
    return m_batt_state;
}

bool battery_is_low(void)
{
    return (m_batt_state == BATT_STATE_LOW);
}

bool battery_is_charging(void)
{
    /* MCP73831 STAT is active-low: pin = 0 → charging */
    return (nrf_gpio_pin_read(CHG_STAT_PIN) == 0);
}

/**
 * Build the complete 8-bit flag field that Student D's app will parse.
 *
 * Bit layout:
 *   bit 0 (0x01) = battery OK   (1 = healthy)
 *   bit 1 (0x02) = battery low  (1 = low)
 *   bit 2 (0x04) = charging     (1 = USB-C charging)
 *   bit 7 (0x80) = fault        (NOT set here — set in main.c)
 *   bits 3-6     = reserved (0)
 */
uint8_t battery_get_flags(void)
{
    uint8_t flags = 0;

    if (m_batt_state == BATT_STATE_OK)
    {
        flags |= FLAG_BATT_OK;
    }
    else if (m_batt_state == BATT_STATE_LOW)
    {
        flags |= FLAG_BATT_LOW;
    }

    if (battery_is_charging())
    {
        flags |= FLAG_CHARGING;
    }

    return flags;
}

/* ====================================================================
 * Legacy API (preserved for backward compatibility)
 * ==================================================================== */

batt_state_t battery_classify(uint16_t mv)
{
    if (mv == 0u)
    {
        return BATT_STATE_UNKNOWN;
    }
    else if (mv < BATT_LOW_MV)
    {
        return BATT_STATE_LOW;
    }
    else
    {
        return BATT_STATE_OK;
    }
}

uint8_t battery_flags_from_state(batt_state_t st)
{
    switch (st)
    {
        case BATT_STATE_OK:
            return FLAG_BATT_OK;

        case BATT_STATE_LOW:
            return FLAG_BATT_LOW;

        case BATT_STATE_UNKNOWN:
        default:
            return 0;
    }
}