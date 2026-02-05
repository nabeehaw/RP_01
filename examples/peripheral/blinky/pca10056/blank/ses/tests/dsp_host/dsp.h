#ifndef DSP_H
#define DSP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================
// Global DSP configuration
// ============================

#define DSP_FS_HZ                2000.0f      // sampling rate [Hz]
#define DSP_WINDOW_MS            250u         // feature window [ms]
#define DSP_WINDOW_SAMPLES       500u         // 0.25 s * 2000 Hz

// One ADC count at ADS1292R input for Vref=2.42 V, G=6
// (already aligned with Student A)
#define DSP_LSB_UV               0.048f       // µV / count

// MDF implementation toggle:
// 0  -> MDF is set to 0.0f (stub, simplest for now).
// 1  -> you implement FFT-based MDF later in dsp_compute_mdf().
#define DSP_ENABLE_MDF_FFT       0

// ============================
// Types
// ============================

// Simple Direct Form I biquad
typedef struct
{
    float b0, b1, b2;   // numerator
    float a1, a2;       // denominator (a0 assumed = 1)
    float z1, z2;       // delay state
} dsp_biquad_t;

// Per-channel DSP state
typedef struct
{
    dsp_biquad_t hp;          // 20 Hz high-pass
    dsp_biquad_t lp;          // 450 Hz low-pass
    dsp_biquad_t notch;       // 60 Hz notch (optional)
    bool         notch_enabled;

    float        window[DSP_WINDOW_SAMPLES];  // last 250 ms of filtered samples
    uint16_t     win_index;
} dsp_channel_t;

// Output feature set for one window
typedef struct
{
    float rms_uV;  // RMS amplitude in µV
    float mdf_Hz;  // median frequency in Hz (0 if MDF disabled)
} dsp_features_t;

// ============================
// API
// ============================

// Initialise a DSP channel with filters and clear state.
// notch_enable: true to run 60 Hz notch, false to skip it.
void dsp_channel_init(dsp_channel_t *ch, bool notch_enable);

// Reset state (window + filter delays) without changing configuration.
void dsp_channel_reset(dsp_channel_t *ch);

// Process one raw ADC sample (24-bit sign-extended) for this channel.
// raw_code: signed ADC code from ADS1292R.
// out_feat: if a new 250 ms window has just completed, features are written here.
// Returns true exactly once per 250 ms window when features are valid.
bool dsp_process_sample(dsp_channel_t *ch, int32_t raw_code, dsp_features_t *out_feat);

#ifdef __cplusplus
}
#endif

#endif // DSP_H
