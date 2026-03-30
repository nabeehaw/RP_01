#include "emg_packet.h"
#include "battery.h"
#include <math.h>

/**
 * Build flags from battery + lead-off.
 * FAULT bit (0x80) is NOT set here — main.c ORs it in if needed.
 */
uint8_t emg_build_flags(uint8_t lead_off)
{
    uint8_t flags = battery_get_flags();

    /* If any electrode is disconnected, set LEAD_OFF flag */
    if (lead_off != 0)
    {
        flags |= EMG_FLAG_LEAD_OFF;
    }

    return flags;
}

static int16_t quantize(float value, float scale)
{
    float q = value / scale;
    if (q >  32767.0f) q =  32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)lrintf(q);
}

void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint16_t seq,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out)
{
    out->seq          = seq;
    out->timestamp_ms = timestamp_ms;
    out->rms_ch1      = quantize(f1->rms_uV, EMG_PACKET_SCALE_RMS_UV);
    out->rms_ch2      = quantize(f2->rms_uV, EMG_PACKET_SCALE_RMS_UV);
    out->mdf_ch1      = quantize(f1->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);
    out->mdf_ch2      = quantize(f2->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);
    out->mpf_ch1      = quantize(f1->mpf_Hz, EMG_PACKET_SCALE_MPF_HZ);
    out->mpf_ch2      = quantize(f2->mpf_Hz, EMG_PACKET_SCALE_MPF_HZ);
    out->flags        = flags;
}