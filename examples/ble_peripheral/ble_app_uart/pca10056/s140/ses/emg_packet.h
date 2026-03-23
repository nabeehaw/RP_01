// emg_packet.h
#pragma once
#include <stdint.h>
#include "dsp.h"

#define EMG_PACKET_SCALE_RMS_UV   0.5f   // 1 LSB = 0.5 µV 
#define EMG_PACKET_SCALE_MDF_HZ   1.0f   // 1 LSB = 1 Hz
#define EMG_FLAG_BATT_OK      0x01
#define EMG_FLAG_BATT_WARN    0x02
#define EMG_FLAG_BATT_CRIT    0x04
#define EMG_FLAG_FAULT        0x80


typedef struct __attribute__((packed))
{
    uint32_t timestamp_ms;   // 4 bytes
    int16_t  rms_ch1;        // scaled RMS
    int16_t  rms_ch2;
    int16_t  mdf_ch1;
    int16_t  mdf_ch2;
    uint8_t  flags;          // battery + status bits
} emg_packet_t;              // 13 bytes total

void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out);
uint8_t emg_build_flags(void);