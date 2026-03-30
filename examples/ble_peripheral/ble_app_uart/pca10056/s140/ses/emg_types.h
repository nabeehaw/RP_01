#ifndef EMG_TYPES_H
#define EMG_TYPES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int32_t ch1;       /* sign-extended 24-bit sample for channel 1     */
    int32_t ch2;       /* sign-extended 24-bit sample for channel 2     */
    uint8_t lead_off;  /* lead-off status bits from ADS1292R status word
                        *   bit 0 = IN1P off
                        *   bit 1 = IN1N off
                        *   bit 2 = IN2P off
                        *   bit 3 = IN2N off
                        *   0x00 = all electrodes connected              */
} emg_sample_t;

#endif /* EMG_TYPES_H */