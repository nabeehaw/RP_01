#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "platform.h"
#include "emg_acquisition.h"

int main(void)
{
    platform_init();
    platform_log("System boot\n");

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

    uint32_t sample_counter = 0u;

    for (;;)
    {
        emg_acquisition_poll();

        emg_sample_t sample;
        while (emg_buffer_pop(&g_emg_buffer, &sample))
        {
            sample_counter++;

            if ((sample_counter % 50u) == 0u)
            {
                platform_log("EMG[%lu]: ch1=%ld ch2=%ld\n",
                             (unsigned long)sample_counter,
                             (long)sample.ch1,
                             (long)sample.ch2);
            }
        }

        platform_delay_ms(1u);
    }
}
