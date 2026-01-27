#include "emg_acquisition.h"
#include "ads1292r.h"
#include "platform.h"

emg_buffer_t g_emg_buffer;

bool emg_acquisition_init(void)
{
    emg_buffer_init(&g_emg_buffer);

    if (!ads1292r_init())
    {
        platform_log("ADS1292R init failed\n");
        return false;
    }

    if (!ads1292r_start_continuous())
    {
        platform_log("ADS1292R start failed\n");
        return false;
    }

    platform_log("EMG acquisition started\n");
    return true;
}

void emg_acquisition_poll(void)
{
    if (!platform_ads1292r_drdy_is_low())
    {
        return; // no new sample yet
    }

    emg_sample_t sample;
    if (ads1292r_read_sample(&sample))
    {
        if (!emg_buffer_push(&g_emg_buffer, &sample))
        {
            // Buffer overflow; you may want to set a flag or drop oldest sample instead
            platform_log("EMG buffer overflow\n");
        }
    }
}
