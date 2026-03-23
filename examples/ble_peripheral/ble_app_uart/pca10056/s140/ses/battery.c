#include "battery.h"
#include "nrf_gpio.h"

// MCP73831 charger STAT net is connected to nRF P0.06
// Active-low:
//   0 = charging
//   1 = not charging / standby
#define CHG_STAT_PIN  NRF_GPIO_PIN_MAP(0, 6)

// Temporary battery thresholds in millivolts
#define BATT_WARN_MV  3600u
#define BATT_CRIT_MV  3400u

void battery_init(void)
{
    // External pull-up resistor already exists on CHG_STAT (R47 to SYS_3V3),
    // so no internal pull resistor is needed.
    nrf_gpio_cfg_input(CHG_STAT_PIN, NRF_GPIO_PIN_NOPULL);
}

uint16_t battery_read_mv(void)
{
    return 0u; // No battery voltage divider/ADC measurement confirmed
}

bool battery_is_charging(void)
{
    // MCP73831 STAT is active-low
    return (nrf_gpio_pin_read(CHG_STAT_PIN) == 0);
}

batt_state_t battery_classify(uint16_t mv)
{
    if (mv == 0u)
    {
        return BATT_STATE_UNKNOWN;
    }
    else if (mv <= BATT_CRIT_MV)
    {
        return BATT_STATE_CRIT;
    }
    else if (mv <= BATT_WARN_MV)
    {
        return BATT_STATE_WARN;
    }
    else
    {
        return BATT_STATE_OK;
    }
}

uint8_t battery_flags_from_state(batt_state_t st)
{
    uint8_t flags = 0u;

    switch (st)
    {
        case BATT_STATE_OK:
            // all flags remain 0
            break;

        case BATT_STATE_WARN:
            // bit 0 = battery low
            flags |= (1u << 0);
            break;

        case BATT_STATE_CRIT:
            // bit 0 = battery low
            // bit 1 = battery critical
            flags |= (1u << 0);
            flags |= (1u << 1);
            break;

        case BATT_STATE_UNKNOWN:
        default:
            // Leave flags at 0 for now
            break;
    }

    return flags;
}