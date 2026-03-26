/**
 * emg_packet.h — BLE feature packet for Kinetic Link
 *
 * Sent every 250 ms (4 Hz) over BLE NUS.
 *
 * Encoding (per capstone report Table 3.2.2):
 *   RMS :  int16, 1 µV/LSB     → code = round(RMS_µV)
 *   MDF :  int16, 0.5 Hz/LSB   → code = round(2 × MDF_Hz)
 *   MPF :  int16, 0.5 Hz/LSB   → code = round(2 × MPF_Hz)
 *
 * Student D decode formulas:
 *   RMS_µV  = (float)rms_chN              (multiply by 1.0)
 *   MDF_Hz  = (float)mdf_chN × 0.5        (divide by 2)
 *   MPF_Hz  = (float)mpf_chN × 0.5        (divide by 2)
 *
 * Flag byte:
 *   bit 0 (0x01) = BATT_OK    — battery healthy
 *   bit 1 (0x02) = BATT_LOW   — battery depleted
 *   bit 2 (0x04) = CHARGING   — USB-C charging
 *   bit 7 (0x80) = FAULT      — EMG init failure / HW error
 *   bits 3-6     = reserved (0)
 */
#pragma once
#include <stdint.h>
#include "dsp.h"

/* ---- Quantisation scales (value / scale = code) ---- */
#define EMG_PACKET_SCALE_RMS_UV   1.0f    /* 1 LSB = 1.0 µV            */
#define EMG_PACKET_SCALE_MDF_HZ   0.5f    /* 1 LSB = 0.5 Hz            */
#define EMG_PACKET_SCALE_MPF_HZ   0.5f    /* 1 LSB = 0.5 Hz            */

/* ---- Flag bit masks ---- */
#define EMG_FLAG_BATT_OK      0x01
#define EMG_FLAG_BATT_LOW     0x02
#define EMG_FLAG_CHARGING     0x04
#define EMG_FLAG_FAULT        0x80

/* ---- Packet structure (17 bytes, packed) ---- */
typedef struct __attribute__((packed))
{
    uint32_t timestamp_ms;    /*  4 B  — ms since boot                  */
    int16_t  rms_ch1;         /*  2 B  — RMS biceps   (1 µV/LSB)       */
    int16_t  rms_ch2;         /*  2 B  — RMS triceps  (1 µV/LSB)       */
    int16_t  mdf_ch1;         /*  2 B  — MDF biceps   (0.5 Hz/LSB)     */
    int16_t  mdf_ch2;         /*  2 B  — MDF triceps  (0.5 Hz/LSB)     */
    int16_t  mpf_ch1;         /*  2 B  — MPF biceps   (0.5 Hz/LSB)     */
    int16_t  mpf_ch2;         /*  2 B  — MPF triceps  (0.5 Hz/LSB)     */
    uint8_t  flags;           /*  1 B  — battery + status               */
} emg_packet_t;               /* total = 17 bytes                       */

/**
 * Build the flags byte from current battery + system state.
 */
uint8_t emg_build_flags(void);

/**
 * Quantise DSP features into a BLE-ready packet.
 */
void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out);