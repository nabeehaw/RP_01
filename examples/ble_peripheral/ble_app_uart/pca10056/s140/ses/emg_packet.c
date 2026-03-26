#include "emg_packet.h"
#include "battery.h"
#include <math.h>

/**
 * Build the flags byte from the battery module's cached state.
 * FAULT bit (0x80) is NOT set here — main.c ORs it in if needed.
 */
uint8_t emg_build_flags(void)
{
    return battery_get_flags();
}

/**
 * Saturating quantise: value / scale → int16, clamped to ±32767.
 */
static int16_t quantize(float value, float scale)
{
    float q = value / scale;
    if (q >  32767.0f) q =  32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)lrintf(q);
}

void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out)
{
    out->timestamp_ms = timestamp_ms;

    /* RMS: 1 µV/LSB  → code = round(RMS_µV / 1.0) */
    out->rms_ch1 = quantize(f1->rms_uV, EMG_PACKET_SCALE_RMS_UV);
    out->rms_ch2 = quantize(f2->rms_uV, EMG_PACKET_SCALE_RMS_UV);

    /* MDF: 0.5 Hz/LSB → code = round(MDF_Hz / 0.5) = round(2 × MDF_Hz) */
    out->mdf_ch1 = quantize(f1->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);
    out->mdf_ch2 = quantize(f2->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);

    /* MPF: 0.5 Hz/LSB → code = round(MPF_Hz / 0.5) = round(2 × MPF_Hz) */
    out->mpf_ch1 = quantize(f1->mpf_Hz, EMG_PACKET_SCALE_MPF_HZ);
    out->mpf_ch2 = quantize(f2->mpf_Hz, EMG_PACKET_SCALE_MPF_HZ);

    out->flags = flags;
}