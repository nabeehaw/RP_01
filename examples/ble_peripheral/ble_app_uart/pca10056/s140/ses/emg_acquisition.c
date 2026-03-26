#include "emg_acquisition.h"
#include "ads1292r.h"
#include "platform.h"

emg_buffer_t g_emg_buffer;

bool emg_acquisition_init(void)
{
    emg_buffer_init(&g_emg_buffer);

    if (!ads1292r_init())
    {
        platform_log("ADS1292R init failed\r\n");
        return false;
    }

    if (!ads1292r_start_continuous())
    {
        platform_log("ADS1292R start continuous failed\r\n");
        return false;
    }

    platform_log("EMG acquisition started\r\n");
    return true;
}

void emg_acquisition_poll(void)
{
    emg_sample_t sample;

    // Drain all ready samples, not just one.
    while (platform_ads1292r_drdy_is_low())
    {
        if (!ads1292r_read_sample(&sample))
        {
            platform_log("ADS1292R sample read failed\r\n");
            break;
        }

        if (!emg_buffer_push(&g_emg_buffer, &sample))
        {
            // Buffer full: drop newest sample and report it.
            platform_log("EMG buffer overflow\r\n");
            break;
        }
    }
}