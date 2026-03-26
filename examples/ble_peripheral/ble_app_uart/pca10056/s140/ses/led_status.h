#ifndef LED_STATUS_H
#define LED_STATUS_H

/**
 * led_status.h — Single-LED status indicator for Kinetic Link
 *
 * All patterns are driven by a 100 ms tick from an app_timer.
 *
 * Mode            Pattern                    Visual meaning
 * ─────────────── ────────────────────────── ─────────────────────────
 * OFF             LED always off             Device sleeping / idle
 * HEARTBEAT       100 ms ON / 900 ms OFF     Normal operation
 * LOW_BATT        100 ms ON / 500 ms OFF     Battery low warning
 * CHARGING        300 ms ON / 300 ms OFF     USB-C charging (overrides others)
 * FAULT           100 ms ON / 100 ms OFF     EMG init failure / error
 *
 * Priority (highest first): FAULT > CHARGING > LOW_BATT > HEARTBEAT > OFF
 * The caller (main.c) is responsible for choosing the correct mode based
 * on system state; the led_status module just executes the pattern.
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LED_MODE_OFF = 0,
    LED_MODE_HEARTBEAT,     /* 100 ms ON  / 900 ms OFF  (1 s period)   */
    LED_MODE_LOW_BATT,      /* 100 ms ON  / 500 ms OFF  (600 ms period) */
    LED_MODE_CHARGING,      /* 300 ms ON  / 300 ms OFF  (600 ms period) */
    LED_MODE_FAULT          /* 100 ms ON  / 100 ms OFF  (200 ms period) */
} led_mode_t;

/* Initialise LED state (LED off). Call after platform_init(). */
void led_status_init(void);

/* Set the active blink pattern. Resets the tick counter. */
void led_status_set_mode(led_mode_t mode);

/* Return the currently active mode. */
led_mode_t led_status_get_mode(void);

/* Call exactly once every 100 ms from the LED app_timer callback. */
void led_status_tick_100ms(void);

#endif /* LED_STATUS_H */