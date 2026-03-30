/**
 * main.c — Kinetic Link firmware
 *
 * BLE NUS + 4 Hz EMG feature extraction + battery monitoring.
 *
 * Compile-time flags:
 *   DIAGNOSTIC_LOG_ENABLED  1 = verbose per-frame RTT logging
 *                           0 = quiet (production)
 *
 * Test sequence (when DIAGNOSTIC_LOG_ENABLED = 1):
 *   Phase 0  "SETTLE"  : IIR filters warming up (first ~2 s, no output)
 *   Phase 1  "REST"    : First 40 valid frames (10 s) — resting baseline
 *                        Summary stats printed at end of phase
 *   Phase 2  "ACTIVE"  : Continuous — flex muscle to see RMS/MDF change
 *
 * LED patterns:
 *   HEARTBEAT : 100 ms ON / 900 ms OFF  — normal
 *   LOW_BATT  : 100 ms ON / 500 ms OFF  — battery low
 *   CHARGING  : 300 ms ON / 300 ms OFF  — USB-C charging (overrides)
 *   FAULT     : 100 ms ON / 100 ms OFF  — EMG init failure
 *
 * BLE packet (17 bytes, sent at 4 Hz):
 *   [0..3]  timestamp_ms  (uint32, little-endian)
 *   [4..5]  rms_ch1       (int16, 1 µV/LSB)
 *   [6..7]  rms_ch2       (int16, 1 µV/LSB)
 *   [8..9]  mdf_ch1       (int16, 0.5 Hz/LSB)
 *   [10..11] mdf_ch2      (int16, 0.5 Hz/LSB)
 *   [12..13] mpf_ch1      (int16, 0.5 Hz/LSB)
 *   [14..15] mpf_ch2      (int16, 0.5 Hz/LSB)
 *   [16]    flags         (uint8, see emg_packet.h)
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

/* ---- Nordic SDK ---- */
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

/* ---- Application modules ---- */
#include "platform.h"
#include "emg_acquisition.h"
#include "emg_buffer.h"
#include "emg_types.h"
#include "dsp.h"
#include "emg_packet.h"
#include "battery.h"
#include "led_status.h"
#include "ads1292r.h"

/* ====================================================================
 * Diagnostic control
 *
 * Set to 1 for bench testing (verbose RTT output every frame).
 * Set to 0 for production / BLE-only operation.
 * ==================================================================== */
#define DIAGNOSTIC_LOG_ENABLED   1

/* ====================================================================
 * BLE configuration
 * ==================================================================== */
#define APP_BLE_CONN_CFG_TAG            1
#define DEVICE_NAME                     "KineticLink"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN
#define APP_BLE_OBSERVER_PRIO           3

#define APP_ADV_INTERVAL                64
#define APP_ADV_DURATION                0

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define DEAD_BEEF                       0xDEADBEEF

#define OPCODE_LENGTH                   1
#define HANDLE_LENGTH                   2

#define LED_TICK_MS                     100u
#define EMG_TICK_MS                     250u

/* ====================================================================
 * BLE instances
 * ==================================================================== */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_emg_timer_id);

/* ====================================================================
 * Module state
 * ==================================================================== */
static dsp_channel_t m_dsp_ch1;
static dsp_channel_t m_dsp_ch2;
static uint32_t      m_emg_frame_counter = 0;
static volatile bool m_emg_tick          = false;
static uint16_t      m_conn_handle       = BLE_CONN_HANDLE_INVALID;
static uint16_t      m_ble_nus_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;

static bool          m_emg_init_ok       = false;

static ble_uuid_t m_adv_uuids[] =
{
    { BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE }
};

/* ====================================================================
 * Diagnostic statistics — cycling 40-frame phases
 *
 * Phase sequence (each 40 frames = 10 seconds):
 *   Phase 0: REST 1     "Rest your arm"
 *   Phase 1: FLEX 1     "Flex now"
 *   Phase 2: REST 2     "Relax again"
 *   Phase 3: FLEX 2     "Flex again"
 *   Phase 4+: FREE      continuous, no prompts
 *
 * A summary is printed at the end of every 40-frame block.
 * ==================================================================== */
#if DIAGNOSTIC_LOG_ENABLED

#define DIAG_PHASE_LEN   40u   /* frames per phase */

typedef struct
{
    float sum;
    float min;
    float max;
    uint32_t count;
} diag_stat_t;

static diag_stat_t m_stat_rms1, m_stat_rms2;
static diag_stat_t m_stat_mdf1, m_stat_mdf2;

static void diag_stat_reset(diag_stat_t *s)
{
    s->sum   = 0.0f;
    s->min   = 1e9f;
    s->max   = -1e9f;
    s->count = 0;
}

static void diag_stat_add(diag_stat_t *s, float v)
{
    s->sum += v;
    if (v < s->min) s->min = v;
    if (v > s->max) s->max = v;
    s->count++;
}

static int32_t diag_avg(const diag_stat_t *s)
{
    if (s->count == 0) return 0;
    return (int32_t)(s->sum / (float)s->count + 0.5f);
}

static int32_t diag_min_i(const diag_stat_t *s)
{
    if (s->count == 0) return 0;
    return (int32_t)(s->min + 0.5f);
}

static int32_t diag_max_i(const diag_stat_t *s)
{
    if (s->count == 0) return 0;
    return (int32_t)(s->max + 0.5f);
}

static void diag_reset_stats(void)
{
    diag_stat_reset(&m_stat_rms1);
    diag_stat_reset(&m_stat_rms2);
    diag_stat_reset(&m_stat_mdf1);
    diag_stat_reset(&m_stat_mdf2);
}

static const char * diag_phase_name(uint32_t phase)
{
    switch (phase)
    {
        case 0: return "REST1 ";
        case 1: return "FLEX1 ";
        case 2: return "REST2 ";
        case 3: return "FLEX2 ";
        default: return "FREE  ";
    }
}

static void diag_print_phase_summary(uint32_t phase)
{
    platform_delay_ms(50);
    const char *name = diag_phase_name(phase);
    uint32_t clean = m_stat_rms1.count;
    uint32_t spikes = DIAG_PHASE_LEN - clean;

    platform_log("\r\n-- %s SUMMARY (%lu clean, %lu spikes rejected) --\r\n",
                 name, (unsigned long)clean, (unsigned long)spikes);
    platform_log("CH1 RMS avg=%ld min=%ld max=%ld uV  MDF avg=%ld Hz\r\n",
                 (long)diag_avg(&m_stat_rms1),
                 (long)diag_min_i(&m_stat_rms1),
                 (long)diag_max_i(&m_stat_rms1),
                 (long)diag_avg(&m_stat_mdf1));
    platform_log("CH2 RMS avg=%ld min=%ld max=%ld uV  MDF avg=%ld Hz\r\n",
                 (long)diag_avg(&m_stat_rms2),
                 (long)diag_min_i(&m_stat_rms2),
                 (long)diag_max_i(&m_stat_rms2),
                 (long)diag_avg(&m_stat_mdf2));

    /* Prompt for next phase */
    switch (phase)
    {
        case 0: platform_log(">> FLEX your muscle NOW <<\r\n\r\n"); break;
        case 1: platform_log(">> RELAX your muscle NOW <<\r\n\r\n"); break;
        case 2: platform_log(">> FLEX again NOW <<\r\n\r\n"); break;
        case 3: platform_log(">> Test complete. Free run. <<\r\n\r\n"); break;
        default: platform_log("\r\n"); break;
    }
}

#endif /* DIAGNOSTIC_LOG_ENABLED */

/* ====================================================================
 * Forward declarations
 * ==================================================================== */
static void timers_init(void);
static void gap_params_init(void);
static void services_init(void);
static void conn_params_init(void);
static void ble_stack_init(void);
static void gatt_init(void);
static void advertising_init(void);
static void log_init(void);
static void power_management_init(void);
static void idle_state_handle(void);
static void advertising_start(void);

/* ====================================================================
 * LED timer callback (100 ms)
 *
 * Priority: FAULT > CHARGING > LOW_BATT > HEARTBEAT
 * Also logs LED mode + CHG pin state every 5 seconds for debugging.
 * ==================================================================== */
static void led_timer_handler(void *p_context)
{
    (void)p_context;
    static uint16_t s_led_log_cnt = 0;

    /* Read CHG_STAT pin directly */
    bool charging = battery_is_charging();
    bool low      = battery_is_low();

    if (!m_emg_init_ok)
    {
        led_status_set_mode(LED_MODE_FAULT);
    }
    else if (charging)
    {
        led_status_set_mode(LED_MODE_CHARGING);
    }
    else if (low)
    {
        led_status_set_mode(LED_MODE_LOW_BATT);
    }
    else
    {
        led_status_set_mode(LED_MODE_HEARTBEAT);
    }

    led_status_tick_100ms();

    /* Log LED state every 5 seconds (50 ticks × 100 ms) */
    s_led_log_cnt++;
    if (s_led_log_cnt >= 50u)
    {
        s_led_log_cnt = 0;
        const char *mode_str;
        switch (led_status_get_mode())
        {
            case LED_MODE_HEARTBEAT: mode_str = "HEARTBEAT"; break;
            case LED_MODE_LOW_BATT:  mode_str = "LOW_BATT";  break;
            case LED_MODE_CHARGING:  mode_str = "CHARGING";  break;
            case LED_MODE_FAULT:     mode_str = "FAULT";     break;
            default:                 mode_str = "OFF";       break;
        }
        platform_log("[LED] mode=%s chg_pin=%u batt_low=%u\r\n",
                     mode_str,
                     (unsigned)charging,
                     (unsigned)low);
    }
}

/* ====================================================================
 * EMG timer callback (250 ms)
 * ==================================================================== */
static void emg_timer_handler(void *p_context)
{
    (void)p_context;
    m_emg_tick = true;
}

/* ====================================================================
 * EMG processing task
 *
 * Called from the main loop when the 250 ms tick fires.
 * Polls ADS1292R, feeds DSP, builds and sends BLE packets.
 * ==================================================================== */
static void emg_task(void)
{
    emg_sample_t   sample;
    dsp_features_t feat1;
    dsp_features_t feat2;
    static uint32_t s_tick_count = 0;

    s_tick_count++;

    /* Refresh battery state RARELY — the nRF52840 SAADC injects analog
     * switching noise into VDD during conversion, which couples into the
     * ADS1292R and causes ~12000 µV spikes.  Reading every 64 ticks
     * (16 seconds) is plenty for battery monitoring.                   */
    if ((s_tick_count % 64u) == 0u)
    {
        battery_update();
    }

    /* Samples are continuously drained by the main loop's
     * emg_acquisition_poll().  Here we just process what's buffered. */

    /* Print buffer level every 4 ticks during settling only */
    size_t buf_count = emg_buffer_count(&g_emg_buffer);
    if ((s_tick_count % 4u) == 1u && m_dsp_ch1.settle_count < DSP_SETTLE_WINDOWS)
    {
        platform_log("[TICK %lu] buf=%u settle1=%u settle2=%u\r\n",
                     (unsigned long)s_tick_count,
                     (unsigned)buf_count,
                     (unsigned)m_dsp_ch1.settle_count,
                     (unsigned)m_dsp_ch2.settle_count);
    }

    /* Process all buffered samples through DSP */
    while (emg_buffer_pop(&g_emg_buffer, &sample))
    {
        bool ready1 = dsp_process_sample(&m_dsp_ch1, sample.ch1, &feat1);
        bool ready2 = dsp_process_sample(&m_dsp_ch2, sample.ch2, &feat2);

        if (ready1 && ready2)
        {
            m_emg_frame_counter++;

            uint32_t ts_ms = m_emg_frame_counter * EMG_TICK_MS;

            /* Build flags */
            uint8_t flags = emg_build_flags();
            if (!m_emg_init_ok)
            {
                flags |= EMG_FLAG_FAULT;
            }

            /* ---- Spike detection (always active, not just diagnostic) ----
             * Skip frames where:
             *   - Either channel RMS exceeds 1500 µV (hardware glitch)
             *   - Either channel MDF > 180 Hz (impossible for real EMG,
             *     indicates CH2 rail clipping or SPI framing error)     */
            #define SPIKE_RMS_THRESH_UV  1500.0f
            #define SPIKE_MDF_THRESH_HZ  180.0f

            bool is_spike = (feat1.rms_uV > SPIKE_RMS_THRESH_UV) ||
                            (feat2.rms_uV > SPIKE_RMS_THRESH_UV) ||
                            (feat1.mdf_Hz > SPIKE_MDF_THRESH_HZ) ||
                            (feat2.mdf_Hz > SPIKE_MDF_THRESH_HZ);

            /* ---- Diagnostic logging (cycling phases) ---- */
#if DIAGNOSTIC_LOG_ENABLED
            {
                /* Frame index within current phase (0-based) */
                uint32_t phase_frame = ((m_emg_frame_counter - 1u) % DIAG_PHASE_LEN);
                uint32_t phase_num   = ((m_emg_frame_counter - 1u) / DIAG_PHASE_LEN);

                /* Reset stats at the start of each new phase */
                if (phase_frame == 0u)
                {
                    diag_reset_stats();

                    /* Print phase header */
                    const char *name = diag_phase_name(phase_num);
                    platform_delay_ms(50);
                    platform_log("--- %s START (frames %lu-%lu) ---\r\n",
                        name,
                        (unsigned long)m_emg_frame_counter,
                        (unsigned long)(m_emg_frame_counter + DIAG_PHASE_LEN - 1u));
                }

                if (!is_spike)
                {
                    /* Clean frame — include in statistics */
                    diag_stat_add(&m_stat_rms1, feat1.rms_uV);
                    diag_stat_add(&m_stat_rms2, feat2.rms_uV);
                    diag_stat_add(&m_stat_mdf1, feat1.mdf_Hz);
                    diag_stat_add(&m_stat_mdf2, feat2.mdf_Hz);
                }

                /* Per-frame log: * prefix = spike (excluded from avg + BLE) */
                platform_log("%s%s F%lu | R1=%ld R2=%ld | M1=%ld M2=%ld\r\n",
                    is_spike ? "*" : " ",
                    diag_phase_name(phase_num),
                    (unsigned long)m_emg_frame_counter,
                    (long)(int32_t)(feat1.rms_uV + 0.5f),
                    (long)(int32_t)(feat2.rms_uV + 0.5f),
                    (long)(int32_t)(feat1.mdf_Hz + 0.5f),
                    (long)(int32_t)(feat2.mdf_Hz + 0.5f));

                /* Print summary at end of phase */
                if (phase_frame == (DIAG_PHASE_LEN - 1u))
                {
                    diag_print_phase_summary(phase_num);
                }
            }
#endif /* DIAGNOSTIC_LOG_ENABLED */

            /* ---- Build and send BLE packet (skip spikes) ---- */
            if (!is_spike && m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                emg_packet_t pkt;
                emg_build_packet(&feat1, &feat2, ts_ms, flags, &pkt);

                uint16_t len = sizeof(pkt);
                uint32_t err = ble_nus_data_send(&m_nus,
                                                 (uint8_t *)&pkt,
                                                 &len,
                                                 m_conn_handle);

                /* Log first successful send so team can confirm data flows */
                static bool s_first_send_logged = false;
                if (err == NRF_SUCCESS && !s_first_send_logged)
                {
                    s_first_send_logged = true;
                    platform_log("[BLE] First packet sent (%u B)\r\n",
                                 (unsigned)sizeof(pkt));
                }

                if ((err != NRF_SUCCESS)          &&
                    (err != NRF_ERROR_RESOURCES)   &&
                    (err != NRF_ERROR_INVALID_STATE) &&
                    (err != NRF_ERROR_NOT_FOUND)   &&
                    (err != NRF_ERROR_BUSY))
                {
                    APP_ERROR_CHECK(err);
                }
            }
        }
    }
}

/* ====================================================================
 * main()
 * ==================================================================== */
int main(void)
{
    ret_code_t err_code;

    /* ---- Hardware FIRST (so platform_log works for BLE debugging) ---- */
    platform_init();

    timers_init();
    log_init();
    power_management_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    platform_log("BLE: OK\r\n");

    /* ---- Remaining hardware ---- */
    battery_init();
    led_status_init();

    /* Immediately report battery + charging pin state */
    platform_log("Batt: %u mV | CHG pin=%u (%s)\r\n",
                 battery_read_mv(),
                 (unsigned)battery_is_charging(),
                 battery_is_charging() ? "CHARGING" : "NOT CHARGING");

    /* ---- DSP channels (60 Hz notch enabled) ---- */
    dsp_channel_init(&m_dsp_ch1, true);
    dsp_channel_init(&m_dsp_ch2, true);

    /* ---- EMG acquisition (ADS1292R over SPI) ---- */
    m_emg_init_ok = emg_acquisition_init();

    if (!m_emg_init_ok)
    {
        NRF_LOG_ERROR("EMG acquisition init FAILED");
        platform_log("ERROR: EMG acquisition init FAILED\r\n");
    }
    else
    {
        NRF_LOG_INFO("EMG acquisition init OK");
        platform_log("EMG acquisition init OK\r\n");
    }

#if DIAGNOSTIC_LOG_ENABLED
    diag_reset_stats();

    /* Let RTT drain the boot messages before printing more */
    platform_delay_ms(200);

    platform_log("\r\n== Diagnostic Mode ==\r\n");
    platform_log("Batt: %u mV | Pkt: %u B\r\n",
                 battery_read_mv(),
                 (unsigned)sizeof(emg_packet_t));

    /* ============================================================
     * SPI FRAME PROBE — REST vs FLEX comparison
     *
     * Pass 1: 100 samples while RELAXED (collect baseline)
     * Wait:   10 seconds (you flex during this)
     * Pass 2: 100 samples while FLEXED (collect active)
     * Compare: if range increases 3× or more, electrodes work
     *
     * All values are RAW — no DSP filtering applied.
     * ============================================================ */
    if (m_emg_init_ok)
    {
        platform_log("\r\n--- SPI PROBE: REST vs FLEX ---\r\n");
        platform_delay_ms(100);

        /* Hex dump first 5 frames for alignment check */
        platform_log("Frame alignment check:\r\n");
        for (uint8_t i = 0; i < 5; i++)
        {
            uint32_t timeout = 20000u;
            while (!platform_ads1292r_drdy_is_low() && timeout > 0) timeout--;
            if (timeout == 0) continue;

            uint8_t frame[9];
            if (ads1292r_read_raw_frame(frame))
            {
                bool stat_ok = ((frame[0] & 0xF0) == 0xC0);
                platform_log("  %u | %02X %02X %02X | %02X %02X %02X | %02X %02X %02X | %s\r\n",
                    i, frame[0],frame[1],frame[2],
                    frame[3],frame[4],frame[5],
                    frame[6],frame[7],frame[8],
                    stat_ok ? "OK" : "BAD");
            }
            platform_delay_ms(5);
        }

        /* Helper: collect 100 raw samples and compute statistics */
        /* We define the stats inline since we need to run it twice */
        typedef struct {
            int32_t min, max;
            int64_t sum;
            uint32_t count;
        } probe_stat_t;

        /* ---- PASS 1: RESTING ---- */
        platform_delay_ms(50);
        platform_log("\r\nPASS 1: RESTING (keep arm relaxed)...\r\n");
        platform_delay_ms(500); /* brief settle */

        probe_stat_t r1 = { .min=0x7FFFFFFF, .max=(int32_t)0x80000000, .sum=0, .count=0 };
        probe_stat_t r2 = { .min=0x7FFFFFFF, .max=(int32_t)0x80000000, .sum=0, .count=0 };

        for (uint16_t i = 0; i < 100; i++)
        {
            uint32_t timeout = 20000u;
            while (!platform_ads1292r_drdy_is_low() && timeout > 0) timeout--;
            if (timeout == 0) continue;

            uint8_t frame[9];
            if (!ads1292r_read_raw_frame(frame)) continue;

            int32_t ch1 = ((int32_t)frame[3]<<16)|((int32_t)frame[4]<<8)|frame[5];
            int32_t ch2 = ((int32_t)frame[6]<<16)|((int32_t)frame[7]<<8)|frame[8];
            if (ch1 & 0x800000) ch1 |= (int32_t)0xFF000000;
            if (ch2 & 0x800000) ch2 |= (int32_t)0xFF000000;

            r1.sum += ch1; if (ch1 < r1.min) r1.min = ch1; if (ch1 > r1.max) r1.max = ch1; r1.count++;
            r2.sum += ch2; if (ch2 < r2.min) r2.min = ch2; if (ch2 > r2.max) r2.max = ch2; r2.count++;
        }

        int32_t r1_range = r1.max - r1.min;
        int32_t r2_range = r2.max - r2.min;
        int32_t r1_uv = (int32_t)((float)r1_range * 0.048f);
        int32_t r2_uv = (int32_t)((float)r2_range * 0.048f);

        platform_log("REST CH1: avg=%ld range=%ld counts (%ld uV pp)\r\n",
            (long)(int32_t)(r1.sum / (int64_t)r1.count), (long)r1_range, (long)r1_uv);
        platform_log("REST CH2: avg=%ld range=%ld counts (%ld uV pp)\r\n",
            (long)(int32_t)(r2.sum / (int64_t)r2.count), (long)r2_range, (long)r2_uv);

        /* ---- WAIT: 10 seconds for user to flex ---- */
        platform_log("\r\n>>> FLEX YOUR BICEPS NOW! Holding for 10 seconds... <<<\r\n");
        platform_delay_ms(50);

        /* Drain any samples during the wait so buffer doesn't overflow */
        for (uint8_t sec = 10; sec > 0; sec--)
        {
            platform_log("  %u...\r\n", sec);
            for (uint16_t t = 0; t < 100; t++)
            {
                platform_delay_ms(10);
                /* drain DRDY samples during wait */
                if (platform_ads1292r_drdy_is_low())
                {
                    uint8_t dump[9];
                    ads1292r_read_raw_frame(dump);
                }
            }
        }
        platform_log("  Capturing FLEXED data now...\r\n");

        /* ---- PASS 2: FLEXED ---- */
        probe_stat_t f1 = { .min=0x7FFFFFFF, .max=(int32_t)0x80000000, .sum=0, .count=0 };
        probe_stat_t f2 = { .min=0x7FFFFFFF, .max=(int32_t)0x80000000, .sum=0, .count=0 };

        for (uint16_t i = 0; i < 100; i++)
        {
            uint32_t timeout = 20000u;
            while (!platform_ads1292r_drdy_is_low() && timeout > 0) timeout--;
            if (timeout == 0) continue;

            uint8_t frame[9];
            if (!ads1292r_read_raw_frame(frame)) continue;

            int32_t ch1 = ((int32_t)frame[3]<<16)|((int32_t)frame[4]<<8)|frame[5];
            int32_t ch2 = ((int32_t)frame[6]<<16)|((int32_t)frame[7]<<8)|frame[8];
            if (ch1 & 0x800000) ch1 |= (int32_t)0xFF000000;
            if (ch2 & 0x800000) ch2 |= (int32_t)0xFF000000;

            f1.sum += ch1; if (ch1 < f1.min) f1.min = ch1; if (ch1 > f1.max) f1.max = ch1; f1.count++;
            f2.sum += ch2; if (ch2 < f2.min) f2.min = ch2; if (ch2 > f2.max) f2.max = ch2; f2.count++;
        }

        int32_t f1_range = f1.max - f1.min;
        int32_t f2_range = f2.max - f2.min;
        int32_t f1_uv = (int32_t)((float)f1_range * 0.048f);
        int32_t f2_uv = (int32_t)((float)f2_range * 0.048f);

        platform_log("FLEX CH1: avg=%ld range=%ld counts (%ld uV pp)\r\n",
            (long)(int32_t)(f1.sum / (int64_t)f1.count), (long)f1_range, (long)f1_uv);
        platform_log("FLEX CH2: avg=%ld range=%ld counts (%ld uV pp)\r\n",
            (long)(int32_t)(f2.sum / (int64_t)f2.count), (long)f2_range, (long)f2_uv);

        /* ---- COMPARISON ---- */
        platform_delay_ms(50);
        platform_log("\r\n=== REST vs FLEX COMPARISON ===\r\n");
        platform_log("  CH1: REST=%ld uV pp  FLEX=%ld uV pp", (long)r1_uv, (long)f1_uv);
        if (r1_uv > 0)
        {
            uint32_t ratio_x10 = (uint32_t)((uint32_t)f1_uv * 10u / (uint32_t)r1_uv);
            platform_log("  (x%lu.%lu)", (unsigned long)(ratio_x10 / 10u),
                                          (unsigned long)(ratio_x10 % 10u));
        }
        platform_log("\r\n");

        platform_log("  CH2: REST=%ld uV pp  FLEX=%ld uV pp", (long)r2_uv, (long)f2_uv);
        if (r2_uv > 0)
        {
            uint32_t ratio_x10 = (uint32_t)((uint32_t)f2_uv * 10u / (uint32_t)r2_uv);
            platform_log("  (x%lu.%lu)", (unsigned long)(ratio_x10 / 10u),
                                          (unsigned long)(ratio_x10 % 10u));
        }
        platform_log("\r\n");

        /* Verdict */
        bool ch1_works = (f1_uv > r1_uv * 2);
        bool ch2_works = (f2_uv > r2_uv * 2);

        platform_log("\r\n  CH1: %s\r\n", ch1_works
            ? "PASS — flex increased signal (electrodes working)"
            : "FAIL — no change with flex (check electrode placement)");
        platform_log("  CH2: %s\r\n", ch2_works
            ? "PASS — flex increased signal (electrodes working)"
            : "FAIL — no change with flex (check electrode placement)");

        if (!ch1_works && !ch2_works)
        {
            platform_log("\r\n  Neither channel responds to flex.\r\n");
            platform_log("  Possible causes:\r\n");
            platform_log("  1. Electrodes too close together (<2 cm)\r\n");
            platform_log("  2. Electrodes on tendon, not muscle belly\r\n");
            platform_log("  3. Dry contact, need gel or wet skin\r\n");
            platform_log("  4. Wires not reaching ADS1292R input pins\r\n");
        }

        platform_log("=== END PROBE ===\r\n\r\n");
        platform_delay_ms(100);
    }

    /* ============================================================
     * SYNTHETIC SIGNAL TEST — verify filters, RMS, MDF, MPF
     *
     * Uses a temporary DSP channel fed with known sine waves.
     * No hardware involved — pure math validation.
     *
     * For each test:
     *   1. Init a fresh DSP channel
     *   2. Feed (settle + 1 window) = 8×500 + 500 = 4500 samples
     *   3. Print expected vs actual
     * ============================================================ */
    {
        platform_log("--- SYNTHETIC FILTER TEST ---\r\n");
        platform_delay_ms(50);

        /* We'll run 5 tests.  For each, allocate a temp DSP channel
         * on the stack (it's ~2 KB but we have plenty). */
        dsp_channel_t  test_ch;
        dsp_features_t test_feat;

        /* Number of samples: settle windows + 1 real window */
        const uint16_t total_samples =
            (DSP_SETTLE_WINDOWS + 1u) * DSP_WINDOW_SAMPLES;

        /* ---- Test 1: 100 Hz pure sine, 10000 counts peak ----
         * Expected after HP(20)+LP(450)+notch(60):
         *   100 Hz is in passband → passes through
         *   RMS = 10000/sqrt(2) × 0.048 = ~339 µV
         *   MDF ≈ 100 Hz,  MPF ≈ 100 Hz                        */
        {
            dsp_channel_init(&test_ch, true);
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                float t = (float)n / DSP_FS_HZ;
                int32_t code = (int32_t)(10000.0f
                    * sinf(2.0f * 3.14159265f * 100.0f * t));

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T1: 100Hz sine 10000pk\r\n");
                platform_log("  Expected: RMS~339uV MDF~100Hz MPF~100Hz\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            else
            {
                platform_log("T1: NO OUTPUT (settling issue)\r\n");
            }
            platform_delay_ms(50);
        }

        /* ---- Test 2: 60 Hz pure sine, 10000 counts peak ----
         * Expected: notch at 60 Hz should kill this.
         *   RMS ≈ very low (< 20 µV)
         *   MDF/MPF meaningless when signal ≈ 0                 */
        {
            dsp_channel_init(&test_ch, true);  /* notch ON */
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                float t = (float)n / DSP_FS_HZ;
                int32_t code = (int32_t)(10000.0f
                    * sinf(2.0f * 3.14159265f * 60.0f * t));

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T2: 60Hz sine 10000pk (notch ON)\r\n");
                platform_log("  Expected: RMS<20uV (notch kills 60Hz)\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            platform_delay_ms(50);
        }

        /* ---- Test 3: 60 Hz sine with notch OFF ----
         * Expected: 60 Hz passes through (HP at 20 Hz, LP at 450 Hz)
         *   RMS ≈ 339 µV (same as test 1)
         *   MDF ≈ 60 Hz,  MPF ≈ 60 Hz                          */
        {
            dsp_channel_init(&test_ch, false);  /* notch OFF */
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                float t = (float)n / DSP_FS_HZ;
                int32_t code = (int32_t)(10000.0f
                    * sinf(2.0f * 3.14159265f * 60.0f * t));

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T3: 60Hz sine 10000pk (notch OFF)\r\n");
                platform_log("  Expected: RMS~339uV MDF~60Hz MPF~60Hz\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            platform_delay_ms(50);
        }

        /* ---- Test 4: 10 Hz sine (below HP cutoff) ----
         * Expected: 20 Hz high-pass should block this
         *   RMS ≈ very low (< 30 µV)                            */
        {
            dsp_channel_init(&test_ch, true);
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                float t = (float)n / DSP_FS_HZ;
                int32_t code = (int32_t)(10000.0f
                    * sinf(2.0f * 3.14159265f * 10.0f * t));

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T4: 10Hz sine 10000pk (below HP)\r\n");
                platform_log("  Expected: RMS<30uV (HP blocks 10Hz)\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            platform_delay_ms(50);
        }

        /* ---- Test 5: Two-tone 80 Hz + 200 Hz, equal amplitude ----
         * Expected:
         *   Both in passband
         *   MDF ≈ between 80 and 200 (closer to geometric mean)
         *   MPF ≈ (80 + 200) / 2 = 140 Hz for equal power       */
        {
            dsp_channel_init(&test_ch, true);
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                float t = (float)n / DSP_FS_HZ;
                int32_t code = (int32_t)(
                    5000.0f * sinf(2.0f * 3.14159265f * 80.0f * t) +
                    5000.0f * sinf(2.0f * 3.14159265f * 200.0f * t));

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T5: 80Hz+200Hz equal 5000pk each\r\n");
                platform_log("  Expected: MDF~120-140Hz MPF~140Hz\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            platform_delay_ms(50);
        }

        /* ---- Test 6: DC offset (constant value) ----
         * Expected: HP filter blocks DC completely
         *   RMS ≈ 0 µV                                          */
        {
            dsp_channel_init(&test_ch, true);
            bool got_result = false;

            for (uint16_t n = 0; n < total_samples; n++)
            {
                int32_t code = 100000;  /* large DC offset */

                if (dsp_process_sample(&test_ch, code, &test_feat))
                {
                    got_result = true;
                }
            }

            if (got_result)
            {
                platform_log("T6: DC offset 100000 counts\r\n");
                platform_log("  Expected: RMS~0uV (HP blocks DC)\r\n");
                platform_log("  Got:      RMS=%ld   MDF=%ld   MPF=%ld\r\n",
                    (long)(int32_t)(test_feat.rms_uV + 0.5f),
                    (long)(int32_t)(test_feat.mdf_Hz + 0.5f),
                    (long)(int32_t)(test_feat.mpf_Hz + 0.5f));
            }
            platform_delay_ms(50);
        }

        platform_log("--- END SYNTHETIC TEST ---\r\n\r\n");
        platform_delay_ms(100);
    }

    platform_log("Phases: 40 frames (10s) each\r\n");
    platform_log("REST1 -> FLEX1 -> REST2 -> FLEX2 -> FREE\r\n");
    platform_log("RELAX muscle for first 10s...\r\n\r\n");
#endif

    /* ---- Timers ---- */
    err_code = app_timer_create(&m_led_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                led_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_led_timer_id,
                               APP_TIMER_TICKS(LED_TICK_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_emg_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                emg_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_emg_timer_id,
                               APP_TIMER_TICKS(EMG_TICK_MS),
                               NULL);
    APP_ERROR_CHECK(err_code);

    /* ---- BLE advertising ---- */
    advertising_start();

    NRF_LOG_INFO("Kinetic Link started.");
    platform_log("INIT DONE. Loop starting.\r\n\r\n");

    /* ---- Superloop ----
     *
     * DRDY fires at 2 kHz.  We MUST poll it on every loop iteration,
     * not just on the 250 ms timer tick.  idle_state_handle() would
     * put the CPU to sleep and miss ~499/500 DRDY pulses.
     *
     * Structure:
     *   1. Always: drain ADS1292R samples into the ring buffer
     *   2. On 250 ms tick: run DSP + build/send BLE packets
     *   3. Process NRF_LOG (non-blocking)
     */
    while (1)
    {
        /* --- Continuously drain ADS1292R at full 2 kHz rate --- */
        emg_acquisition_poll();

        /* --- On 250 ms tick: process buffered samples through DSP --- */
        if (m_emg_tick)
        {
            m_emg_tick = false;
            emg_task();
        }

        /* Process pending log entries (non-blocking, does NOT sleep) */
        NRF_LOG_PROCESS();
    }
}

/* ====================================================================
 * BLE boilerplate (unchanged)
 * ==================================================================== */

void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void nus_data_handler(ble_nus_evt_t *p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_DEBUG("Received data from BLE NUS.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data,
                              p_evt->params.rx_data.length);
    }
}

static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle,
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Advertising");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("Advertising idle");
            break;
        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    uint32_t err_code;
    (void)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            platform_log("[BLE] CONNECTED\r\n");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            platform_log("[BLE] DISCONNECTED\r\n");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(
                p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(
                m_conn_handle,
                BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gattc_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(
                p_ble_evt->evt.gatts_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO,
                          ble_evt_handler, NULL);
}

static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt,
                             nrf_ble_gatt_evt_t const *p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len =
            p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("MTU updated. NUS max data len = %u",
                     m_ble_nus_max_data_len);
    }
    (void)p_gatt;
}

static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt,
                                               NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              =
        BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt =
        sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler                  = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    if (err_code != NRF_SUCCESS)
    {
        return;
    }

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void power_management_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    if (!NRF_LOG_PROCESS())
    {
        nrf_pwr_mgmt_run();
    }
}

static void advertising_start(void)
{
    uint32_t err_code =
        ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    if (err_code == NRF_SUCCESS)
    {
        platform_log("[BLE] Advertising as '%s'\r\n", DEVICE_NAME);
    }
    else
    {
        platform_log("[BLE] Adv FAILED err=%lu (EMG still runs)\r\n",
                     (unsigned long)err_code);
    }
}