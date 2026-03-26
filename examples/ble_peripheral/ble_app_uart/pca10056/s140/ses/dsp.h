#ifndef DSP_H
#define DSP_H

/**
 * dsp.h — On-device filtering and feature extraction for Kinetic Link
 *
 * Pipeline per channel:
 *   raw 24-bit ADC code → HP 20 Hz → LP 450 Hz → Notch 60 Hz → window[500]
 *
 * Every 250 ms (500 samples at 2 kHz), features are computed:
 *   - RMS  (µV)  — time-domain amplitude
 *   - MDF  (Hz)  — median frequency (fatigue marker)
 *   - MPF  (Hz)  — mean power frequency (fatigue marker)
 *
 * Fs = 2000 Hz, window = 250 ms = 500 samples
 * FFT length = 512 (zero-padded), frequency resolution ≈ 3.9 Hz
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ====================================================================
 * Configuration
 * ==================================================================== */

#define DSP_FS_HZ                2000.0f     /* sampling rate [Hz]       */
#define DSP_WINDOW_MS            250u        /* feature window [ms]      */
#define DSP_WINDOW_SAMPLES       500u        /* 0.25 s × 2000 Hz        */

/* ADC LSB for ADS1292R: Vref = 2.42 V, Gain = 6
 *   full-scale = ±2.42/6 = ±403.33 mV
 *   LSB = 403333 µV / 2^23 = 0.04807 µV/count                        */
#define DSP_LSB_UV               0.048f      /* µV / count              */

/* MDF/MPF implementation: always enabled now.
 * Uses self-contained radix-2 FFT (no CMSIS dependency).              */
#define DSP_ENABLE_MDF_FFT       1

/* FFT parameters */
#define DSP_FFT_LEN              512u        /* must be power of 2      */
#define DSP_FFT_BINS             (DSP_FFT_LEN / 2u)  /* 256 one-sided  */

/* Frequency bins for the sEMG band (20–450 Hz)
 *   bin = freq × FFT_LEN / Fs
 *   20 Hz  → ceil(20 × 512 / 2000)  = 6   (~23.4 Hz)
 *   450 Hz → floor(450 × 512 / 2000) = 115 (~449.2 Hz)               */
#define DSP_BIN_LOW              6u
#define DSP_BIN_HIGH             115u

/* Number of startup windows to discard while IIR filters settle.
 * 8 windows × 250 ms = 2 seconds of settling time.                    */
#define DSP_SETTLE_WINDOWS       8u

/* ====================================================================
 * Types
 * ==================================================================== */

/* Direct Form II transposed biquad */
typedef struct
{
    float b0, b1, b2;      /* numerator coefficients                    */
    float a1, a2;          /* denominator (a0 assumed = 1)              */
    float z1, z2;          /* delay states                              */
} dsp_biquad_t;

/* Per-channel DSP state */
typedef struct
{
    dsp_biquad_t hp;                              /* 20 Hz high-pass    */
    dsp_biquad_t lp;                              /* 450 Hz low-pass    */
    dsp_biquad_t notch;                           /* 60 Hz notch        */
    bool         notch_enabled;

    float        window[DSP_WINDOW_SAMPLES];      /* filtered samples   */
    uint16_t     win_index;                        /* current write pos  */
    uint16_t     settle_count;                     /* windows discarded  */
} dsp_channel_t;

/* Output features for one 250 ms window */
typedef struct
{
    float rms_uV;       /* RMS amplitude in µV                          */
    float mdf_Hz;       /* median frequency in Hz                       */
    float mpf_Hz;       /* mean power frequency in Hz                   */
} dsp_features_t;

/* ====================================================================
 * Public API
 * ==================================================================== */

/**
 * Initialise a DSP channel: set filter coefficients, clear window.
 * @param notch_enable  true to include 60 Hz notch in the chain.
 */
void dsp_channel_init(dsp_channel_t *ch, bool notch_enable);

/**
 * Reset filter delays and window without changing coefficients.
 */
void dsp_channel_reset(dsp_channel_t *ch);

/**
 * Feed one raw ADC sample into the filter chain and window.
 * @param raw_code  24-bit signed ADC code from ADS1292R.
 * @param out_feat  pointer to receive features when a window completes.
 * @return true exactly once per 250 ms when features are valid.
 *         Returns false during the settling period (first 2 s).
 */
bool dsp_process_sample(dsp_channel_t *ch,
                        int32_t raw_code,
                        dsp_features_t *out_feat);

/**
 * Returns true once the channel has passed the IIR settling period
 * and is producing valid feature data.
 */
bool dsp_is_settled(const dsp_channel_t *ch);

#ifdef __cplusplus
}
#endif

#endif /* DSP_H */