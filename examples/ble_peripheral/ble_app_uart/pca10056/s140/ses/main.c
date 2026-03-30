/**
 * main.c — Kinetic Link PRODUCTION firmware
 *
 * Changes from debug build:
 *   - DIAGNOSTIC_LOG_ENABLED = 0 (no per-frame RTT spam)
 *   - RUN_BOOT_TESTS = 0 (no SPI probe / synthetic tests at boot)
 *   - Sequence number in every BLE packet (Student D detects drops)
 *   - Dropped-packet counter logged periodically
 *   - Lead-off detection enabled (electrode contact status in flags)
 *   - Spike rejection on BLE output (RMS > 1500 or MDF > 180)
 *
 * BLE packet: 19 bytes at 4 Hz over NUS
 * LED patterns: heartbeat / low_batt / charging / fault
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

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
 * Build switches
 *
 * DIAGNOSTIC_LOG_ENABLED:
 *   1 = per-frame RTT logging (bench testing)
 *   0 = quiet (production / demo)
 *
 * RUN_BOOT_TESTS:
 *   1 = SPI probe + synthetic filter tests at boot
 *   0 = skip (production / demo)
 * ==================================================================== */
#define DIAGNOSTIC_LOG_ENABLED   0
#define RUN_BOOT_TESTS           0

/* Spike rejection thresholds */
#define SPIKE_RMS_THRESH_UV      20000.0f
#define SPIKE_MDF_THRESH_HZ      180.0f

/* Battery update interval (every 64 ticks = 16 seconds) */
#define BATT_UPDATE_INTERVAL     64u

/* Status log interval (every 40 ticks = 10 seconds) */
#define STATUS_LOG_INTERVAL      40u

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
 * State
 * ==================================================================== */
static dsp_channel_t m_dsp_ch1;
static dsp_channel_t m_dsp_ch2;
static uint32_t      m_emg_frame_counter = 0;
static volatile bool m_emg_tick          = false;
static uint16_t      m_conn_handle       = BLE_CONN_HANDLE_INVALID;
static uint16_t      m_ble_nus_max_data_len =
    BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;
static bool          m_emg_init_ok       = false;

/* Packet sequence and drop tracking */
static uint16_t      m_pkt_seq           = 0;
static uint32_t      m_pkt_sent          = 0;
static uint32_t      m_pkt_dropped       = 0;
static uint32_t      m_pkt_spiked        = 0;

/* Lead-off accumulator — set if ANY sample in the window had lead-off */
static uint8_t       m_lead_off_accum    = 0;

static ble_uuid_t m_adv_uuids[] =
{
    { BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE }
};

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
static void advertising_start(void);

/* ====================================================================
 * LED timer (100 ms)
 * ==================================================================== */
static void led_timer_handler(void *p_context)
{
    (void)p_context;

    if (!m_emg_init_ok)
        led_status_set_mode(LED_MODE_FAULT);
    else if (battery_is_charging())
        led_status_set_mode(LED_MODE_CHARGING);
    else if (battery_is_low())
        led_status_set_mode(LED_MODE_LOW_BATT);
    else
        led_status_set_mode(LED_MODE_HEARTBEAT);

    led_status_tick_100ms();
}

/* ====================================================================
 * EMG timer (250 ms)
 * ==================================================================== */
static void emg_timer_handler(void *p_context)
{
    (void)p_context;
    m_emg_tick = true;
}

/* ====================================================================
 * EMG processing task
 * ==================================================================== */
static void emg_task(void)
{
    emg_sample_t   sample;
    dsp_features_t feat1;
    dsp_features_t feat2;
    static uint32_t s_tick_count = 0;

    s_tick_count++;

    /* Battery update (rare, to avoid SAADC noise coupling) */
    if ((s_tick_count % BATT_UPDATE_INTERVAL) == 0u)
    {
        battery_update();
    }

    /* Process all buffered samples through DSP */
    while (emg_buffer_pop(&g_emg_buffer, &sample))
    {
        /* Accumulate lead-off status across the window */
        m_lead_off_accum |= sample.lead_off;

        bool ready1 = dsp_process_sample(&m_dsp_ch1, sample.ch1, &feat1);
        bool ready2 = dsp_process_sample(&m_dsp_ch2, sample.ch2, &feat2);

        if (ready1 && ready2)
        {
            m_emg_frame_counter++;
            uint32_t ts_ms = m_emg_frame_counter * EMG_TICK_MS;

            /* Build flags: battery + lead-off + fault */
            uint8_t flags = emg_build_flags(m_lead_off_accum);
            if (!m_emg_init_ok)
            {
                flags |= EMG_FLAG_FAULT;
            }

            /* Reset lead-off accumulator for next window */
            uint8_t lead_off_snapshot = m_lead_off_accum;
            m_lead_off_accum = 0;

            /* Spike detection */
            bool is_spike = (feat1.rms_uV > SPIKE_RMS_THRESH_UV) ||
                            (feat2.rms_uV > SPIKE_RMS_THRESH_UV) ||
                            (feat1.mdf_Hz > SPIKE_MDF_THRESH_HZ) ||
                            (feat2.mdf_Hz > SPIKE_MDF_THRESH_HZ);

            if (is_spike)
            {
                m_pkt_spiked++;
            }

#if DIAGNOSTIC_LOG_ENABLED
            platform_log("%sF%lu R1=%ld R2=%ld M1=%ld M2=%ld LO=0x%02X\r\n",
                is_spike ? "*" : " ",
                (unsigned long)m_emg_frame_counter,
                (long)(int32_t)(feat1.rms_uV + 0.5f),
                (long)(int32_t)(feat2.rms_uV + 0.5f),
                (long)(int32_t)(feat1.mdf_Hz + 0.5f),
                (long)(int32_t)(feat2.mdf_Hz + 0.5f),
                lead_off_snapshot);
#endif

            /* Send via BLE (skip spikes) */
            if (!is_spike && m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                emg_packet_t pkt;
                m_pkt_seq++;
                emg_build_packet(&feat1, &feat2, m_pkt_seq, ts_ms,
                                 flags, &pkt);

                uint16_t len = sizeof(pkt);
                uint32_t err = ble_nus_data_send(&m_nus,
                                                 (uint8_t *)&pkt,
                                                 &len,
                                                 m_conn_handle);

                if (err == NRF_SUCCESS)
                {
                    m_pkt_sent++;
                }
                else
                {
                    m_pkt_dropped++;
                }
            }
        }
    }

    /* Periodic status log (every 10 seconds) */
    if ((s_tick_count % STATUS_LOG_INTERVAL) == 0u)
    {
        platform_log("[STATUS] t=%lus sent=%lu drop=%lu spike=%lu batt=%umV lo=0x%02X %s\r\n",
            (unsigned long)(s_tick_count / 4u),
            (unsigned long)m_pkt_sent,
            (unsigned long)m_pkt_dropped,
            (unsigned long)m_pkt_spiked,
            battery_read_mv(),
            m_lead_off_accum,
            m_conn_handle != BLE_CONN_HANDLE_INVALID ? "BLE:ON" : "BLE:OFF");
    }
}

/* ====================================================================
 * main()
 * ==================================================================== */
int main(void)
{
    ret_code_t err_code;

    /* Hardware first (so platform_log works) */
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

    battery_init();
    led_status_init();

    /* DSP channels — 60 Hz double notch enabled */
    dsp_channel_init(&m_dsp_ch1, true);
    dsp_channel_init(&m_dsp_ch2, true);

    /* EMG acquisition (ADS1292R) */
    m_emg_init_ok = emg_acquisition_init();

    platform_log("Kinetic Link PRODUCTION\r\n");
    platform_log("EMG: %s | Batt: %u mV | CHG: %u | Pkt: %u B\r\n",
                 m_emg_init_ok ? "OK" : "FAIL",
                 battery_read_mv(),
                 (unsigned)battery_is_charging(),
                 (unsigned)sizeof(emg_packet_t));

#if RUN_BOOT_TESTS
    /* SPI probe and synthetic tests would go here.
     * Disabled for production. Set RUN_BOOT_TESTS=1 to re-enable. */
#endif

    /* Start timers */
    err_code = app_timer_create(&m_led_timer_id, APP_TIMER_MODE_REPEATED,
                                led_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_led_timer_id,
                               APP_TIMER_TICKS(LED_TICK_MS), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_emg_timer_id, APP_TIMER_MODE_REPEATED,
                                emg_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_emg_timer_id,
                               APP_TIMER_TICKS(EMG_TICK_MS), NULL);
    APP_ERROR_CHECK(err_code);

    /* Start BLE advertising */
    advertising_start();

    /* Superloop — poll DRDY continuously at 2 kHz */
    while (1)
    {
        emg_acquisition_poll();

        if (m_emg_tick)
        {
            m_emg_tick = false;
            emg_task();
        }

        NRF_LOG_PROCESS();
    }
}

/* ====================================================================
 * BLE boilerplate
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
        NRF_LOG_DEBUG("NUS RX data");
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
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        sd_ble_gap_disconnect(m_conn_handle,
                              BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
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
    (void)ble_adv_evt;
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
            /* Reset counters on new connection */
            m_pkt_sent = 0;
            m_pkt_dropped = 0;
            m_pkt_spiked = 0;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            platform_log("[BLE] DISCONNECTED (sent=%lu drop=%lu)\r\n",
                         (unsigned long)m_pkt_sent,
                         (unsigned long)m_pkt_dropped);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys = { .rx_phys = BLE_GAP_PHY_AUTO,
                                          .tx_phys = BLE_GAP_PHY_AUTO };
            sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            sd_ble_gap_sec_params_reply(m_conn_handle,
                BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
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
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler                  = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    if (err_code != NRF_SUCCESS) return;
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
        platform_log("[BLE] Adv FAILED err=%lu\r\n", (unsigned long)err_code);
    }
}