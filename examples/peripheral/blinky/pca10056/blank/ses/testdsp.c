// test_dsp.c
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "dsp.h"

// helper: degrees to radians
static inline float two_pi_f(void) { return 6.28318530717958647692f; }

static void test_sine_100uV_100Hz(void)
{
    printf("=== Test 1: 100 uV, 100 Hz sine ===\n");

    dsp_channel_t ch;
    dsp_channel_init(&ch, true); // notch on

    const float fs      = (float)DSP_FS_HZ;    // 2000 Hz
    const float freq    = 100.0f;             // 100 Hz
    const float amp_uV  = 100.0f;             // 100 µV amplitude
    const float amp_cnt = amp_uV / DSP_LSB_UV; // convert µV -> ADC counts

    const uint32_t total_samples = 10u * DSP_WINDOW_SAMPLES; // 10 windows

    dsp_features_t feat;
    uint32_t frame_idx = 0u;

    for (uint32_t n = 0; n < total_samples; ++n)
    {
        float t   = (float)n / fs;
        float val = amp_cnt * sinf(two_pi_f() * freq * t);   // in counts
        int32_t raw_code = (int32_t)val;

        bool ready = dsp_process_sample(&ch, raw_code, &feat);
        if (ready)
        {
            frame_idx++;
            printf("Frame %2lu: RMS = %.2f uV (MDF=%.1f Hz)\n",
                   (unsigned long)frame_idx,
                   feat.rms_uV, feat.mdf_Hz);
        }
    }

    printf("Expected RMS ≈ %.2f uV\n", amp_uV / (float)sqrt(2.0));
    printf("\n");
}

static void test_dc_1000_counts(void)
{
    printf("=== Test 2: DC input, 1000 counts ===\n");

    dsp_channel_t ch;
    dsp_channel_init(&ch, true); // notch on
    dsp_channel_reset(&ch);

    const int32_t dc_code        = 1000;      // constant input
    const uint32_t total_samples = 10u * DSP_WINDOW_SAMPLES;

    dsp_features_t feat;
    uint32_t frame_idx = 0u;

    for (uint32_t n = 0; n < total_samples; ++n)
    {
        bool ready = dsp_process_sample(&ch, dc_code, &feat);
        if (ready)
        {
            frame_idx++;
            printf("Frame %2lu: RMS = %.4f uV (MDF=%.1f Hz)\n",
                   (unsigned long)frame_idx,
                   feat.rms_uV, feat.mdf_Hz);
        }
    }

    printf("For a proper 20 Hz HPF, RMS should decay towards ~0 uV\n");
    printf("\n");
}

int main(void)
{
    test_sine_100uV_100Hz();
    test_dc_1000_counts();
    return 0;
}
