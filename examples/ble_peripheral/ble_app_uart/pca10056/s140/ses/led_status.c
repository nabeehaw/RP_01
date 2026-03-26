#include "led_status.h"
#include "platform.h"

static led_mode_t m_led_mode = LED_MODE_OFF;
static uint8_t    m_tick     = 0;

void led_status_init(void)
{
    /* platform_init() should already have configured the LED pin. */
    platform_led_set(false);
    m_led_mode = LED_MODE_OFF;
    m_tick     = 0;
}

void led_status_set_mode(led_mode_t mode)
{
    if (mode != m_led_mode)
    {
        m_led_mode = mode;
        m_tick     = 0;          /* restart pattern from the beginning */
    }
}

led_mode_t led_status_get_mode(void)
{
    return m_led_mode;
}

/**
 * Call from the 100 ms app_timer interrupt.
 *
 * Each tick = 100 ms.  Patterns:
 *
 *   HEARTBEAT : period = 10 ticks (1000 ms)
 *               ON for tick 0 (100 ms), OFF for ticks 1-9 (900 ms)
 *
 *   LOW_BATT  : period =  6 ticks (600 ms)
 *               ON for tick 0 (100 ms), OFF for ticks 1-5 (500 ms)
 *
 *   CHARGING  : period =  6 ticks (600 ms)
 *               ON for ticks 0-2 (300 ms), OFF for ticks 3-5 (300 ms)
 *
 *   FAULT     : period =  2 ticks (200 ms)
 *               ON for tick 0 (100 ms), OFF for tick 1 (100 ms)
 */
void led_status_tick_100ms(void)
{
    m_tick++;

    switch (m_led_mode)
    {
        case LED_MODE_OFF:
            platform_led_set(false);
            break;

        case LED_MODE_HEARTBEAT:
            /* 100 ms ON / 900 ms OFF  →  10-tick period */
            if (m_tick >= 10)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick == 0);
            break;

        case LED_MODE_LOW_BATT:
            /* 100 ms ON / 500 ms OFF  →  6-tick period */
            if (m_tick >= 6)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick == 0);
            break;

        case LED_MODE_CHARGING:
            /* 300 ms ON / 300 ms OFF  →  6-tick period */
            if (m_tick >= 6)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick < 3);
            break;

        case LED_MODE_FAULT:
            /* 100 ms ON / 100 ms OFF  →  2-tick period */
            if (m_tick >= 2)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick == 0);
            break;

        default:
            platform_led_set(false);
            break;
    }
}