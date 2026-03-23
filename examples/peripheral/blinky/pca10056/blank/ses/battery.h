typedef enum {
    BATT_STATE_UNKNOWN = 0,
    BATT_STATE_OK,
    BATT_STATE_WARN,
    BATT_STATE_CRIT,
} batt_state_t;

uint16_t battery_read_mv(void);   // returns millivolts (to be implemented later)
batt_state_t battery_classify(uint16_t mv);
uint8_t battery_flags_from_state(batt_state_t st);
