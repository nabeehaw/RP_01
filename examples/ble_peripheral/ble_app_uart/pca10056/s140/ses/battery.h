#ifndef BATTERY_H
#define BATTERY_H

/**
 * battery.h - Battery monitoring for Kinetic Link
 *
 * Hardware:
 *   - nRF52840 VDD is supplied by a TLV700-3.3V LDO fed from the Li-ion cell.
 *     VDD reads ~3300 mV while the LDO regulates, and drops below that only
 *     when the cell itself falls below ~3.5 V (LDO dropout ~0.15 V).
 *   - MCP73831 charger STAT output is on P0.06, active-low:
 *       0 = charging in progress
 *       1 = charge complete / no USB / standby
 *   - R47 (10 kOhm) provides a hardware pull-up on CHG_STAT to SYS_3V3.
 *
 * VDD thresholds (post-LDO, NOT raw battery voltage):
 *   OK  :  VDD >= 3200 mV   (LDO regulating, battery > ~3.4 V)
 *   LOW :  VDD <  3100 mV   (LDO in dropout, battery < ~3.3 V)
 *   Recovery hysteresis: LOW -> OK requires VDD >= 3200 mV
 *
 * Flag byte layout for BLE packets (Student D interface):
 *   bit 0 (0x01) : battery OK   (1 = good)
 *   bit 1 (0x02) : battery low  (1 = low)
 *   bit 2 (0x04) : charging     (1 = USB-C charging in progress)
 *   bit 7 (0x80) : fault        (set externally by main if EMG init fails)
 *   bits 3-6     : reserved (0)
 */

#include <stdint.h>
#include <stdbool.h>

/* ---- Battery state (voltage-based) ---- */
typedef enum {
    BATT_STATE_UNKNOWN = 0, /* ADC not yet read or read failed             */
    BATT_STATE_OK,          /* VDD healthy, LDO regulating                 */
    BATT_STATE_LOW,         /* VDD below threshold, battery nearly empty   */
} batt_state_t;

/* ---- Initialisation ---- */
void battery_init(void);

/**
 * Call periodically (recommended every 250 ms from the main loop) to
 * refresh the cached VDD reading and update the battery state machine
 * with hysteresis.
 */
void battery_update(void);

/* ---- Queries (use cached state, safe to call from ISR context) ---- */
uint16_t     battery_read_mv(void);      /* Averaged SAADC VDD reading      */
batt_state_t battery_get_state(void);    /* Cached state with hysteresis     */
bool         battery_is_low(void);       /* true when state == LOW           */
bool         battery_is_charging(void);  /* MCP73831 STAT pin, real-time     */

/**
 * Build the complete 8-bit flag field for BLE packets.
 * Combines voltage state + charging pin into a single byte.
 */
uint8_t battery_get_flags(void);

/* ---- Legacy API (kept for emg_packet.c compatibility) ---- */
batt_state_t battery_classify(uint16_t mv);
uint8_t      battery_flags_from_state(batt_state_t st);

#endif /* BATTERY_H */