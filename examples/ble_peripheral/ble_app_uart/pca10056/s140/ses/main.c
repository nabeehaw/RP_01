/**
 * Clean custom-board main.c for Kinetic Link
 * - BLE NUS advertising
 * - 4 Hz EMG processing loop
 * - Single status LED:
 *      charging  -> blink 300 ms on / 300 ms off
 *      heartbeat -> blink 100 ms on / 900 ms off
 *      fault     -> blink 100 ms on / 100 ms off
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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

#include "platform.h"
#include "emg_acquisition.h"
#include "emg_buffer.h"
#include "emg_types.h"
#include "dsp.h"
#include "emg_packet.h"
#include "battery.h"

#define APP_BLE_CONN_CFG_TAG            1
#define DEVICE_NAME                     "Nordic_UART"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN
#define APP_BLE_OBSERVER_PRIO           3

#define APP_ADV_INTERVAL                64
#define APP_ADV_DURATION                0   // Advertise forever

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

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

APP_TIMER_DEF(m_led_timer_id);
APP_TIMER_DEF(m_emg_timer_id);

static dsp_channel_t m_dsp_ch1;
static dsp_channel_t m_dsp_ch2;
static uint32_t      m_emg_frame_counter = 0;
static volatile bool m_emg_tick = false;
static uint16_t      m_conn_handle = BLE_CONN_HANDLE_INVALID;
static uint16_t      m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;

static bool          m_emg_init_ok = false;
static uint8_t       m_led_tick = 0;

static ble_uuid_t m_adv_uuids[] =
{
    { BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE }
};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
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

static void nus_data_handler(ble_nus_evt_t * p_evt)
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

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
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

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    (void)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
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

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) &&
        (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("MTU updated. NUS max data len = %u", m_ble_nus_max_data_len);
    }

    NRF_LOG_DEBUG("ATT MTU exchange complete.");
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
    APP_ERROR_CHECK(err_code);

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
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

static void emg_timer_handler(void * p_context)
{
    (void)p_context;
    m_emg_tick = true;
}

static void led_timer_handler(void * p_context)
{
    (void)p_context;
    m_led_tick++;

    if (!m_emg_init_ok)
    {
        // Fault blink: 100 ms on / 100 ms off
        if (m_led_tick >= 2)
        {
            m_led_tick = 0;
        }
        platform_led_set(m_led_tick == 0);
        return;
    }

    if (battery_is_charging())
    {
        // Charging blink: 300 ms on / 300 ms off
        if (m_led_tick >= 6)
        {
            m_led_tick = 0;
        }
        platform_led_set(m_led_tick < 3);
    }
    else
    {
        // Heartbeat: 100 ms on / 900 ms off
        if (m_led_tick >= 10)
        {
            m_led_tick = 0;
        }
        platform_led_set(m_led_tick == 0);
    }
}

static void emg_task(void)
{
    emg_sample_t   sample;
    dsp_features_t feat1;
    dsp_features_t feat2;

    emg_acquisition_poll();

    while (emg_buffer_pop(&g_emg_buffer, &sample))
    {
        bool ready1 = dsp_process_sample(&m_dsp_ch1, sample.ch1, &feat1);
        bool ready2 = dsp_process_sample(&m_dsp_ch2, sample.ch2, &feat2);

        if (ready1 && ready2)
        {
            emg_packet_t pkt;
            uint32_t timestamp_ms = m_emg_frame_counter * EMG_TICK_MS;
            uint8_t  flags        = emg_build_flags();

            emg_build_packet(&feat1, &feat2, timestamp_ms, flags, &pkt);
            m_emg_frame_counter++;

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                uint16_t len = sizeof(pkt);
                uint32_t err = ble_nus_data_send(&m_nus,
                                                 (uint8_t *)&pkt,
                                                 &len,
                                                 m_conn_handle);

                if ((err != NRF_SUCCESS) &&
                    (err != NRF_ERROR_RESOURCES) &&
                    (err != NRF_ERROR_INVALID_STATE) &&
                    (err != NRF_ERROR_NOT_FOUND) &&
                    (err != NRF_ERROR_BUSY))
                {
                    APP_ERROR_CHECK(err);
                }
            }
        }
    }
}

int main(void)
{
    ret_code_t err_code;

    timers_init();
    log_init();
    power_management_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    platform_init();
    battery_init();

    dsp_channel_init(&m_dsp_ch1, true);
    dsp_channel_init(&m_dsp_ch2, true);

    m_emg_init_ok = emg_acquisition_init();

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

    advertising_start();
    NRF_LOG_INFO("Kinetic Link firmware started.");

    while (1)
    {
        if (m_emg_tick)
        {
            m_emg_tick = false;
            emg_task();
        }

        idle_state_handle();
    }
    while(1)
    {
      if (battery_is_charging())
      {
        platform_led_set(true);
        platform_delay_ms(300);
        platform_led_set(false);
        platform_delay_ms(300);
        platform_log("BATTERY: CHARGING\r\n");
      }
      else
      {
      platform_led_set(true);
      platform_delay_ms(100);
      platform_led_set(false);
      platform_delay_ms(900);
      platform_log("BATTERY: NOT CHARGING\r\n");
      }
      platform_delay_ms(1000);
    }
}