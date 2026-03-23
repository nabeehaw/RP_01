#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LED_MODE_OFF = 0,
    LED_MODE_HEARTBEAT,
    LED_MODE_CHARGING,
    LED_MODE_FAULT
} led_mode_t;

void led_status_init(void);
void led_status_set_mode(led_mode_t mode);
void led_status_tick_100ms(void);

#endif