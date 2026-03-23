#include "led_status.h"
#include "platform.h"

static led_mode_t m_led_mode = LED_MODE_OFF;
static uint8_t    m_tick = 0;

void led_status_init(void)
{
    // platform_init() should already have configured the LED pin.
    platform_led_set(false);
    m_led_mode = LED_MODE_OFF;
    m_tick = 0;
}

void led_status_set_mode(led_mode_t mode)
{
    m_led_mode = mode;
    m_tick = 0;
}

void led_status_tick_100ms(void)
{
    m_tick++;

    switch (m_led_mode)
    {
        case LED_MODE_OFF:
            platform_led_set(false);
            break;

        case LED_MODE_HEARTBEAT:
            // Short blink every 1 second:
            // ON for 100 ms, OFF for 900 ms
            if (m_tick >= 10)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick == 0);
            break;

        case LED_MODE_CHARGING:
            // Charging blink:
            // ON 300 ms, OFF 300 ms
            if (m_tick >= 6)
            {
                m_tick = 0;
            }
            platform_led_set(m_tick < 3);
            break;

        case LED_MODE_FAULT:
            // Fast blink:
            // ON 100 ms, OFF 100 ms
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