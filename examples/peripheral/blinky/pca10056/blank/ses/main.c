#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "platform.h"
#include "emg_acquisition.h"
#include "emg_acquisition.h"
#include "emg_buffer.h"
#include "emg_types.h"
#include "dsp.h"

#define LOG_RAW_SAMPLES       0
#define RAW_SAMPLE_DECIMATE   50u //if LOG_RAW_SAMPLES ==1, print every nth sample (avoid flooding)

static dsp_channel_t  g_dsp_ch1;
static dsp_channel_t  g_dsp_ch2;

int main(void)
{
    platform_init();
    platform_log("System boot\n");

    // Initialize dsp for ch1 and ch2, notch ON by default
    dsp_channel_init(&g_dsp_ch1, true);
    dsp_channel_init(&g_dsp_ch2, true);

    // Start emg acquisition (ads1292r and buffer)
    if (!emg_acquisition_init())
    {
        platform_log("EMG acquisition init failed, halting\n");
        while (1)
        {
            platform_led_set(true);
            platform_delay_ms(200u);
            platform_led_set(false);
            platform_delay_ms(200u);
        }
    }

    platform_log("EMG acquisition has started\n");

    uint32_t raw_sample_counter = 0u;
    uint32_t feature_frame_counter = 0u;

    for (;;)
    {
        emg_acquisition_poll();

        emg_sample_t sample;
        while (emg_buffer_pop(&g_emg_buffer, &sample))
        {
            raw_sample_counter++;
            #if LOG_RAW_SAMPLES
            // Raw bring-up path (prints every nth sample)
            if ((raw_sample_counter % RAW_SAMPLE_DECIMATE) == 0u)
            {
                platform_log("RAW[%lu]: ch1=%ld ch2=%ld\n",
                             (unsigned long)raw_sample_counter,
                             (long)sample.ch1,
                             (long)sample.ch2);
            }
            #endif

            dsp_features_t feat1;
            dsp_features_t feat2;

            bool frame_ready_ch1 = dsp_process_sample(&g_dsp_ch1, sample.ch1, &feat1);
            (void)dsp_process_sample(&g_dsp_ch2, sample.ch2, &feat2);

            if (frame_ready_ch1)
            {
              feature_frame_counter++;

              platform_log("FRAME[%lu]: "
                           "ch1_RMS = %.2f uV, ch1_MDF = %.1f Hz | "
                           "ch2_RMS = %.2f uV, ch2_MDF = %.1f Hz\n",
                           (unsigned long)feature_frame_counter,
                           feat1.rms_uV, feat1.mdf_Hz,
                           feat2.rms_uV, feat2.mdf_Hz);
            }

        }

        platform_delay_ms(1u); // small sleep to yield CPU
    }
}
