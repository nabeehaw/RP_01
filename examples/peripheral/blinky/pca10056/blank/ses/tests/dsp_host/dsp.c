#include "dsp.h"

#include <math.h>
#include <string.h>

// ============================
// Filter coefficients (Fs = 2000 Hz)
// Designed as 2nd-order Butterworths and a 60 Hz notch.
// Form: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
//                 - a1*y[n-1] - a2*y[n-2]
// a0 is assumed = 1.
// ============================

// 20 Hz high-pass, Butterworth, order 2, Fs = 2000 Hz
// b:  [ 0.95654323, -1.91308645,  0.95654323 ]
// a:  [ 1.0,        -1.91119707,  0.91497583 ]
static const float HP_B0 =  0.95654323f;
static const float HP_B1 = -1.91308645f;
static const float HP_B2 =  0.95654323f;
static const float HP_A1 = -1.91119707f;
static const float HP_A2 =  0.91497583f;

// 450 Hz low-pass, Butterworth, order 2, Fs = 2000 Hz
// b:  [ 0.24834108,  0.49668216,  0.24834108 ]
// a:  [ 1.0,        -0.18421380,  0.17757812 ]
static const float LP_B0 = 0.24834108f;
static const float LP_B1 = 0.49668216f;
static const float LP_B2 = 0.24834108f;
static const float LP_A1 = -0.18421380f;
static const float LP_A2 = 0.17757812f;

// 60 Hz notch, Q ~ 25, Fs = 2000 Hz (biquad)
// b: [ 0.99624423, -1.95719601, 0.99624423 ]
// a: [ 1.0,        -1.95719601, 0.99248846 ]
static const float NOTCH_B0 =  0.99624423f;
static const float NOTCH_B1 = -1.95719601f;
static const float NOTCH_B2 =  0.99624423f;
static const float NOTCH_A1 = -1.95719601f;
static const float NOTCH_A2 =  0.99248846f;

// ============================
// Internal helpers
// ============================

static void biquad_init(dsp_biquad_t *bq,
                        float b0, float b1, float b2,
                        float a1, float a2)
{
    bq->b0 = b0;
    bq->b1 = b1;
    bq->b2 = b2;
    bq->a1 = a1;
    bq->a2 = a2;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

static inline float biquad_process_sample(dsp_biquad_t *bq, float x)
{
    // Direct Form I
    float y = bq->b0 * x + bq->z1;
    bq->z1 = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2 = bq->b2 * x - bq->a2 * y;
    return y;
}

static void biquad_reset(dsp_biquad_t *bq)
{
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

// Compute features (RMS in µV and MDF in Hz) from a full window.
static void dsp_compute_features(const float *window, dsp_features_t *out_feat)
{
    // RMS in ADC counts
    double sum_sq = 0.0;
    for (uint16_t i = 0; i < DSP_WINDOW_SAMPLES; ++i)
    {
        float v = window[i];
        sum_sq += (double)v * (double)v;
    }

    float rms_counts = 0.0f;
    if (DSP_WINDOW_SAMPLES > 0)
    {
        rms_counts = (float)sqrt(sum_sq / (double)DSP_WINDOW_SAMPLES);
    }

    // Convert to µV
    out_feat->rms_uV = rms_counts * DSP_LSB_UV;

#if DSP_ENABLE_MDF_FFT
    // TODO: implement FFT-based MDF here using CMSIS or my own FFT.
    // Set to 0 for initial build purposes.
    out_feat->mdf_Hz = 0.0f;
#else
    // MDF disabled for now
    out_feat->mdf_Hz = 0.0f;
#endif
}

// ============================
// Public API
// ============================

void dsp_channel_init(dsp_channel_t *ch, bool notch_enable)
{
    if (ch == NULL)
    {
        return;
    }

    biquad_init(&ch->hp,    HP_B0,    HP_B1,    HP_B2,    HP_A1,    HP_A2);
    biquad_init(&ch->lp,    LP_B0,    LP_B1,    LP_B2,    LP_A1,    LP_A2);
    biquad_init(&ch->notch, NOTCH_B0, NOTCH_B1, NOTCH_B2, NOTCH_A1, NOTCH_A2);

    ch->notch_enabled = notch_enable;
    ch->win_index     = 0u;
    memset(ch->window, 0, sizeof(ch->window));
}

void dsp_channel_reset(dsp_channel_t *ch)
{
    if (ch == NULL)
    {
        return;
    }

    biquad_reset(&ch->hp);
    biquad_reset(&ch->lp);
    biquad_reset(&ch->notch);

    ch->win_index = 0u;
    memset(ch->window, 0, sizeof(ch->window));
}

bool dsp_process_sample(dsp_channel_t *ch, int32_t raw_code, dsp_features_t *out_feat)
{
    if (ch == NULL)
    {
        return false;
    }

    // Convert raw ADC counts to float. We stay in counts internally;
    // scaling to µV is done at the RMS stage.
    float x = (float)raw_code;

    // Filter chain: HP -> LP -> (optional) notch
    float y = biquad_process_sample(&ch->hp, x);
    y       = biquad_process_sample(&ch->lp, y);
    if (ch->notch_enabled)
    {
        y = biquad_process_sample(&ch->notch, y);
    }

    // Store into current 250 ms window
    ch->window[ch->win_index] = y;
    ch->win_index++;

    // If window not yet full, nothing to output.
    if (ch->win_index < DSP_WINDOW_SAMPLES)
    {
        return false;
    }

    // Window just completed: compute features and reset index.
    ch->win_index = 0u;

    if (out_feat != NULL)
    {
        dsp_compute_features(ch->window, out_feat);
    }

    return true;
}
