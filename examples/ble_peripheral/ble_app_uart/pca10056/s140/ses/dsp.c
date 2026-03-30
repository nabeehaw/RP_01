#include "dsp.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ====================================================================
 * Filter coefficients (Fs = 2000 Hz)
 *
 * Direct Form II transposed biquad:
 *   y[n] = b0·x[n] + z1
 *   z1   = b1·x[n] − a1·y[n] + z2
 *   z2   = b2·x[n] − a2·y[n]
 * ==================================================================== */

/* 20 Hz high-pass, 2nd-order Butterworth */
static const float HP_B0 =  0.95654323f;
static const float HP_B1 = -1.91308645f;
static const float HP_B2 =  0.95654323f;
static const float HP_A1 = -1.91119707f;
static const float HP_A2 =  0.91497583f;

/* 450 Hz low-pass, 2nd-order Butterworth */
static const float LP_B0 =  0.24834108f;
static const float LP_B1 =  0.49668216f;
static const float LP_B2 =  0.24834108f;
static const float LP_A1 = -0.18421380f;
static const float LP_A2 =  0.17757812f;

/* 60 Hz notch, Q ≈ 25 */
static const float NOTCH_B0 =  0.99624423f;
static const float NOTCH_B1 = -1.95719601f;
static const float NOTCH_B2 =  0.99624423f;
static const float NOTCH_A1 = -1.95719601f;
static const float NOTCH_A2 =  0.99248846f;

/* ====================================================================
 * FFT working buffers (shared between channels — only one computes
 * at a time since everything runs in the main loop, never from ISR)
 * ==================================================================== */

#if DSP_ENABLE_MDF_FFT

static float s_fft_re[DSP_FFT_LEN];     /* real part        (2048 B)  */
static float s_fft_im[DSP_FFT_LEN];     /* imaginary part   (2048 B)  */

/* Pre-computed Hanning window coefficients for 500 samples.
 * Computed once on first call to avoid repeated cosf().               */
static float  s_hann[DSP_WINDOW_SAMPLES];
static bool   s_hann_ready = false;

static void hann_init(void)
{
    if (s_hann_ready)
    {
        return;
    }
    const float inv_n = 1.0f / (float)(DSP_WINDOW_SAMPLES - 1u);
    for (uint16_t i = 0; i < DSP_WINDOW_SAMPLES; i++)
    {
        s_hann[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * (float)i * inv_n));
    }
    s_hann_ready = true;
}

/* ====================================================================
 * Self-contained radix-2 DIT complex FFT (in-place, iterative)
 *
 * re[] and im[] must both have length n, where n is a power of 2.
 * Twiddle factors use recursive multiplication (O(log n) trig calls).
 * ==================================================================== */
static void fft_complex(float *re, float *im, uint16_t n)
{
    uint16_t j = 0;

    /* --- Bit-reversal permutation --- */
    for (uint16_t i = 0; i < n - 1u; i++)
    {
        if (i < j)
        {
            float t;
            t = re[i];  re[i] = re[j];  re[j] = t;
            t = im[i];  im[i] = im[j];  im[j] = t;
        }
        uint16_t k = n >> 1;
        while (k <= j)
        {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    /* --- Butterfly stages --- */
    for (uint16_t half = 1; half < n; half <<= 1)
    {
        uint16_t step = half << 1;
        float theta = -M_PI / (float)half;
        float wpr   = cosf(theta);
        float wpi   = sinf(theta);

        for (uint16_t group = 0; group < n; group += step)
        {
            float wr = 1.0f, wi = 0.0f;
            for (uint16_t p = 0; p < half; p++)
            {
                uint16_t a = group + p;
                uint16_t b = a + half;

                float tr = wr * re[b] - wi * im[b];
                float ti = wr * im[b] + wi * re[b];

                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;

                float new_wr = wr * wpr - wi * wpi;
                wi = wr * wpi + wi * wpr;
                wr = new_wr;
            }
        }
    }
}

/* ====================================================================
 * Compute MDF and MPF from a 500-sample filtered window.
 *
 * Steps:
 *   1. Apply Hanning window to 500 samples
 *   2. Zero-pad to 512 samples
 *   3. 512-point complex FFT (imaginary input = 0)
 *   4. Compute one-sided power spectrum over the 20–450 Hz band
 *   5. MDF = frequency where cumulative power reaches 50%
 *   6. MPF = power-weighted mean frequency
 *
 * Returns MDF and MPF in Hz via pointers.
 * ==================================================================== */
static void dsp_compute_spectral_features(const float *window,
                                          float *out_mdf_Hz,
                                          float *out_mpf_Hz)
{
    /* 1. Apply Hanning window and copy to FFT buffer */
    for (uint16_t i = 0; i < DSP_WINDOW_SAMPLES; i++)
    {
        s_fft_re[i] = window[i] * s_hann[i];
    }
    /* Zero-pad the remaining 12 samples (512 - 500) */
    for (uint16_t i = DSP_WINDOW_SAMPLES; i < DSP_FFT_LEN; i++)
    {
        s_fft_re[i] = 0.0f;
    }
    /* Imaginary part is zero for real input */
    memset(s_fft_im, 0, sizeof(s_fft_im));

    /* 2. 512-point complex FFT */
    fft_complex(s_fft_re, s_fft_im, DSP_FFT_LEN);

    /* 3. Compute total power and weighted-frequency sum in band.
     *    Frequency resolution = Fs / FFT_LEN = 2000 / 512 ≈ 3.906 Hz
     *    Bin k → freq = k × 3.906 Hz
     *
     *    We restrict to the sEMG band: bins 6..115 (≈23–449 Hz).
     *    Two passes:
     *      Pass 1: total power + weighted frequency sum (for MPF)
     *      Pass 2: cumulative power to find MDF                       */
    const float freq_res = DSP_FS_HZ / (float)DSP_FFT_LEN;

    float total_power  = 0.0f;
    float weighted_sum = 0.0f;

    /* Pass 1 — accumulate total power and frequency-weighted power */
    for (uint16_t k = DSP_BIN_LOW; k <= DSP_BIN_HIGH; k++)
    {
        float pk = s_fft_re[k] * s_fft_re[k]
                 + s_fft_im[k] * s_fft_im[k];
        total_power  += pk;
        weighted_sum += pk * ((float)k * freq_res);
    }

    /* Guard: if there's essentially no signal energy, return 0 */
    if (total_power < 1e-10f)
    {
        *out_mdf_Hz = 0.0f;
        *out_mpf_Hz = 0.0f;
        return;
    }

    /* MPF = Σ(f·P(f)) / Σ(P(f)) */
    *out_mpf_Hz = weighted_sum / total_power;

    /* Pass 2 — find MDF (cumulative power reaches 50%) */
    float half_power  = total_power * 0.5f;
    float cumulative  = 0.0f;

    for (uint16_t k = DSP_BIN_LOW; k <= DSP_BIN_HIGH; k++)
    {
        float pk = s_fft_re[k] * s_fft_re[k]
                 + s_fft_im[k] * s_fft_im[k];
        cumulative += pk;

        if (cumulative >= half_power)
        {
            /* Linear interpolation between this bin and the previous
             * for sub-bin MDF accuracy (optional refinement).         */
            float prev_cum = cumulative - pk;
            float frac     = (pk > 1e-12f)
                           ? (half_power - prev_cum) / pk
                           : 0.5f;
            *out_mdf_Hz = ((float)k - 1.0f + frac) * freq_res;
            return;
        }
    }

    /* Should not reach here, but return top of band */
    *out_mdf_Hz = (float)DSP_BIN_HIGH * freq_res;
}

#endif /* DSP_ENABLE_MDF_FFT */

/* ====================================================================
 * Biquad helpers
 * ==================================================================== */

static void biquad_init(dsp_biquad_t *bq,
                        float b0, float b1, float b2,
                        float a1, float a2)
{
    bq->b0 = b0;  bq->b1 = b1;  bq->b2 = b2;
    bq->a1 = a1;  bq->a2 = a2;
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

static inline float biquad_process(dsp_biquad_t *bq, float x)
{
    float y  = bq->b0 * x + bq->z1;
    bq->z1   = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2   = bq->b2 * x - bq->a2 * y;
    return y;
}

static void biquad_reset(dsp_biquad_t *bq)
{
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

/* ====================================================================
 * Compute all features for one completed 250 ms window
 * ==================================================================== */
static void dsp_compute_features(const float *window,
                                 dsp_features_t *out)
{
    /* --- RMS in ADC counts, then convert to µV --- */
    double sum_sq = 0.0;
    for (uint16_t i = 0; i < DSP_WINDOW_SAMPLES; i++)
    {
        double v = (double)window[i];
        sum_sq += v * v;
    }

    float rms_counts = (DSP_WINDOW_SAMPLES > 0u)
        ? (float)sqrt(sum_sq / (double)DSP_WINDOW_SAMPLES)
        : 0.0f;

    out->rms_uV = rms_counts * DSP_LSB_UV;

    /* --- Spectral features (MDF + MPF) --- */
#if DSP_ENABLE_MDF_FFT
    dsp_compute_spectral_features(window, &out->mdf_Hz, &out->mpf_Hz);
#else
    out->mdf_Hz = 0.0f;
    out->mpf_Hz = 0.0f;
#endif
}

/* ====================================================================
 * Public API
 * ==================================================================== */

void dsp_channel_init(dsp_channel_t *ch, bool notch_enable)
{
    if (ch == NULL)
    {
        return;
    }

    biquad_init(&ch->hp,    HP_B0,    HP_B1,    HP_B2,    HP_A1,    HP_A2);
    biquad_init(&ch->lp,    LP_B0,    LP_B1,    LP_B2,    LP_A1,    LP_A2);
    biquad_init(&ch->notch, NOTCH_B0, NOTCH_B1, NOTCH_B2, NOTCH_A1, NOTCH_A2);
    biquad_init(&ch->notch2, NOTCH_B0, NOTCH_B1, NOTCH_B2, NOTCH_A1, NOTCH_A2);

    ch->notch_enabled = notch_enable;
    ch->win_index     = 0u;
    ch->settle_count  = 0u;
    memset(ch->window, 0, sizeof(ch->window));

#if DSP_ENABLE_MDF_FFT
    hann_init();   /* precompute Hanning coefficients (once globally) */
#endif
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
    biquad_reset(&ch->notch2);

    ch->win_index    = 0u;
    ch->settle_count = 0u;
    memset(ch->window, 0, sizeof(ch->window));
}

bool dsp_is_settled(const dsp_channel_t *ch)
{
    if (ch == NULL)
    {
        return false;
    }
    return (ch->settle_count >= DSP_SETTLE_WINDOWS);
}

bool dsp_process_sample(dsp_channel_t *ch,
                        int32_t raw_code,
                        dsp_features_t *out_feat)
{
    if (ch == NULL)
    {
        return false;
    }

    /* Convert 24-bit signed code to float (stays in ADC counts) */
    float x = (float)raw_code;

    /* Filter chain: HP → LP → (optional) notch × 2
     * Cascading two identical notch stages gives ~80 dB rejection
     * at 60 Hz instead of ~40 dB from a single stage.            */
    float y = biquad_process(&ch->hp, x);
    y       = biquad_process(&ch->lp, y);
    if (ch->notch_enabled)
    {
        y = biquad_process(&ch->notch, y);
        y = biquad_process(&ch->notch2, y);
    }

    /* Store into current window */
    ch->window[ch->win_index] = y;
    ch->win_index++;

    /* Window not yet full */
    if (ch->win_index < DSP_WINDOW_SAMPLES)
    {
        return false;
    }

    /* --- Window complete --- */
    ch->win_index = 0u;

    /* Settling guard: discard first DSP_SETTLE_WINDOWS windows
     * to let the IIR filters reach steady state.                      */
    if (ch->settle_count < DSP_SETTLE_WINDOWS)
    {
        ch->settle_count++;
        if (out_feat != NULL)
        {
            out_feat->rms_uV = 0.0f;
            out_feat->mdf_Hz = 0.0f;
            out_feat->mpf_Hz = 0.0f;
        }
        return false;   /* signal: "not ready yet" */
    }

    /* Compute features from the completed window */
    if (out_feat != NULL)
    {
        dsp_compute_features(ch->window, out_feat);
    }

    return true;
}