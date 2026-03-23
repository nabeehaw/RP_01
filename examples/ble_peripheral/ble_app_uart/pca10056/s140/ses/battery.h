#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BATT_STATE_UNKNOWN = 0,
    BATT_STATE_OK,
    BATT_STATE_WARN,
    BATT_STATE_CRIT,
} batt_state_t;

// Initialize battery-related GPIO/status inputs
void battery_init(void);

// Returns battery voltage in millivolts (or a stubbed value for now)
uint16_t    battery_read_mv(void);

// Tells us if battery is charging
bool battery_is_charging(void);

// Classify battery state from millivolts
batt_state_t battery_classify(uint16_t mv);

// Map battery state to 8-bit flag field
uint8_t     battery_flags_from_state(batt_state_t st);

#endif // BATTERY_H
