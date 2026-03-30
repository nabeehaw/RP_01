/**
 * emg_packet.h — BLE feature packet for Kinetic Link (PRODUCTION)
 *
 * 19 bytes, sent at 4 Hz over BLE NUS.
 *
 * Student D decode (little-endian):
 *   Offset  Size  Field          Decode
 *   0       2     seq            packet sequence number (wraps at 65535)
 *   2       4     timestamp_ms   ms since boot
 *   6       2     rms_ch1        × 1.0 = µV
 *   8       2     rms_ch2        × 1.0 = µV
 *   10      2     mdf_ch1        × 0.5 = Hz
 *   12      2     mdf_ch2        × 0.5 = Hz
 *   14      2     mpf_ch1        × 0.5 = Hz
 *   16      2     mpf_ch2        × 0.5 = Hz
 *   18      1     flags          bitmask (see below)
 *
 * Flags byte:
 *   bit 0 (0x01) = BATT_OK      battery healthy
 *   bit 1 (0x02) = BATT_LOW     battery depleted
 *   bit 2 (0x04) = CHARGING     USB-C charging
 *   bit 3 (0x08) = LEAD_OFF     one or more electrodes disconnected
 *   bit 7 (0x80) = FAULT        EMG init failure / HW error
 *   bits 4-6     = reserved (0)
 */
#pragma once
#include <stdint.h>
#include "dsp.h"

/* ---- Quantisation scales ---- */
#define EMG_PACKET_SCALE_RMS_UV   1.0f
#define EMG_PACKET_SCALE_MDF_HZ   0.5f
#define EMG_PACKET_SCALE_MPF_HZ   0.5f

/* ---- Flag bit masks ---- */
#define EMG_FLAG_BATT_OK      0x01
#define EMG_FLAG_BATT_LOW     0x02
#define EMG_FLAG_CHARGING     0x04
#define EMG_FLAG_LEAD_OFF     0x08
#define EMG_FLAG_FAULT        0x80

/* ---- Packet structure (19 bytes, packed) ---- */
typedef struct __attribute__((packed))
{
    uint16_t seq;             /*  2 B  — wrapping sequence number       */
    uint32_t timestamp_ms;    /*  4 B  — ms since boot                  */
    int16_t  rms_ch1;         /*  2 B  — RMS biceps   (1 µV/LSB)       */
    int16_t  rms_ch2;         /*  2 B  — RMS triceps  (1 µV/LSB)       */
    int16_t  mdf_ch1;         /*  2 B  — MDF biceps   (0.5 Hz/LSB)     */
    int16_t  mdf_ch2;         /*  2 B  — MDF triceps  (0.5 Hz/LSB)     */
    int16_t  mpf_ch1;         /*  2 B  — MPF biceps   (0.5 Hz/LSB)     */
    int16_t  mpf_ch2;         /*  2 B  — MPF triceps  (0.5 Hz/LSB)     */
    uint8_t  flags;           /*  1 B  — battery + lead-off + status    */
} emg_packet_t;               /* total = 19 bytes                       */

/**
 * Build flags from battery state + lead-off status.
 * @param lead_off  raw lead-off byte from emg_sample_t (0 = all connected)
 */
uint8_t emg_build_flags(uint8_t lead_off);

/**
 * Quantise DSP features into a BLE-ready packet.
 */
void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint16_t seq,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out);