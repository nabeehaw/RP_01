#include "emg_packet.h"
#include <math.h>

static int16_t quantize(float value, float scale)
{
    float q = value / scale;
    if (q > 32767.0f) q = 32767.0f;
    if (q < -32768.0f) q = -32768.0f;
    return (int16_t)lrintf(q);
}

uint8_t emg_build_flags(void)
{
    // TEMP: until VBAT ADC is implemented, report UNKNOWN/OK
    return EMG_FLAG_BATT_OK;
}

void emg_build_packet(const dsp_features_t *f1,
                      const dsp_features_t *f2,
                      uint32_t timestamp_ms,
                      uint8_t flags,
                      emg_packet_t *out)
{
    out->timestamp_ms = timestamp_ms;
    out->rms_ch1      = quantize(f1->rms_uV, EMG_PACKET_SCALE_RMS_UV);
    out->rms_ch2      = quantize(f2->rms_uV, EMG_PACKET_SCALE_RMS_UV);
    out->mdf_ch1      = quantize(f1->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);
    out->mdf_ch2      = quantize(f2->mdf_Hz, EMG_PACKET_SCALE_MDF_HZ);
    out->flags        = flags;
}
