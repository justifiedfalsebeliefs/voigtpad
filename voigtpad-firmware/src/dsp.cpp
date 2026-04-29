/**
 * @file dsp.cpp
 * @brief Voigtpad audio engine — see dsp.h.
 *
 * Performance-critical practices:
 *   • Audio-rate smoothing (one-pole) on every continuous parameter so the
 *     UI/audio interface is click- and zipper-noise free.
 *   • LFO and filter coefficients that change only at sub-audio rates are
 *     re-computed once per block, not per sample.
 *   • All trig in the inner loop uses sinf/cosf (libm on Cortex-M7 with
 *     hardware FPU).  The chord low-pass is static (2200 Hz) so its
 *     biquad coefficients are computed exactly once at Init.
 *   • No allocations after Init.  No exceptions.
 */

#include "dsp.h"

#include "daisy_seed.h"
#include "hardware.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace VoigtpadDsp {

/* ─────────────────────────────────────────────────────────────────── */
/*  Constants                                                          */
/* ─────────────────────────────────────────────────────────────────── */

static constexpr float kTwoPi   = 6.28318530717958647692f;
static constexpr float kInvSR   = 1.0f / kSampleRate;

/* Master output gain.  Calibrated so that the summed bus
 *   chord(0.75) + sub(0.75) + shmr(0.75)
 * lands just at digital-full-scale before the soft clipper.  At the
 * realistic worst-case peak per section (≈ 1.0 when the signal isn't
 * cancelling itself between voices) the sum is ≈ 2.25; 1/2.25 ≈ 0.444
 * is the headroom-perfect coefficient.  We pick 0.44 to leave one
 * sample of guard band, then SoftClip swallows anything above ~1.0.   */
static constexpr float kMasterGain = 0.44f;

/* Faust patch's iv3/iv5/iv7 tables, full 8-entry version preserved so
 * future spec changes can re-extend the chord menu without rewriting
 * the engine.  The 5-option spec maps onto indices 0,1,2,4,5.        */
static constexpr int8_t kIv3[8] = { 3, 4, 3, 4, 2, 5, 3, 4 };
static constexpr int8_t kIv5[8] = { 7, 7, 7, 7, 7, 7, 6, 8 };
static constexpr int8_t kIv7[8] = {12,12,10,11,12,12,12,12 };

/* User-facing chord index → original Faust index. */
static constexpr uint8_t kChordMap[kNumChords] = {
    0, /* min  */
    1, /* maj  */
    2, /* min7 */
    4, /* sus2 */
    5, /* sus4 */
};

/* Sub octave (Faust default = -1, static for voigtpad). */
static constexpr int8_t kSubOctave = -1;

/* One-pole smoothing coefficients (per-sample).  Roughly 30 ms. */
static constexpr float kSmoothFast = 0.0010f;   /* ~21 ms */
static constexpr float kSmoothSlow = 0.0005f;   /* ~42 ms */

/* ─────────────────────────────────────────────────────────────────── */
/*  Math helpers                                                       */
/* ─────────────────────────────────────────────────────────────────── */

static inline float Midi2Hz(float n)
{
    /* 440 * 2^((n - 69)/12) */
    return 440.0f * std::exp2((n - 69.0f) * (1.0f / 12.0f));
}

static inline float SoftClip(float x)
{
    /* Padé approximant of tanh — same shape as athanor's feedback clip,
     * monotone and C¹.  Output saturates near ±1.                    */
    if (x >  1.5f) return  1.0f;
    if (x < -1.5f) return -1.0f;
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* Phase increment (per sample) for a given frequency. */
static inline float PhaseInc(float hz) { return hz * kInvSR; }

/* Wrap a phase in [0, 1).  Faster than fmodf for a value already close. */
static inline float WrapPhase(float p)
{
    if (p >= 1.0f) p -= std::floor(p);
    else if (p < 0.0f) p -= std::floor(p);
    return p;
}

/* Naive saw in [-1, 1] from 0..1 phase.  No anti-aliasing — the
 * downstream 2200 Hz Butterworth low-pass in the chord section
 * suppresses the worst harmonics, matching the Faust patch's spectral
 * footprint.                                                          */
static inline float Saw(float ph) { return 2.0f * ph - 1.0f; }

/* Sin from 0..1 phase. */
static inline float SinNorm(float ph) { return std::sin(kTwoPi * ph); }

/* slowLFO advance: returns 0.5*sin(2π·rate·t) + 0.5*sin(2π·rate·1.37·t).
 * State pointer must be 2 floats in row-major (ph0, ph1).             */
static inline float SlowLfoTick(float* phs, float rate_hz)
{
    const float inc0 = PhaseInc(rate_hz);
    const float inc1 = PhaseInc(rate_hz * 1.37f);
    phs[0] += inc0; if (phs[0] >= 1.0f) phs[0] -= 1.0f;
    phs[1] += inc1; if (phs[1] >= 1.0f) phs[1] -= 1.0f;
    return 0.5f * SinNorm(phs[0]) + 0.5f * SinNorm(phs[1]);
}

/* Pan: returns the (left_gain, right_gain) pair for an equal-power
 * pan with p in [0, 1].                                               */
static inline void PanGains(float p, float& gl, float& gr)
{
    if (p < 0.0f) p = 0.0f;
    if (p > 1.0f) p = 1.0f;
    gl = std::sqrt(1.0f - p);
    gr = std::sqrt(p);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Biquad helpers (transposed Direct Form II — single-precision)      */
/* ─────────────────────────────────────────────────────────────────── */
/*
 * Common low-pass biquad design (RBJ cookbook, Q = sqrt(2)/2 for
 * Butterworth).  All coefficient computation is done off the audio
 * thread (Init() or once per block) so the hot loop only runs the
 * 5-MAC inner update.
 */
struct BiquadCoeffs
{
    float b0, b1, b2;
    float a1, a2;
};

static BiquadCoeffs DesignLpfButter(float cutoff_hz)
{
    if (cutoff_hz < 20.0f)              cutoff_hz = 20.0f;
    if (cutoff_hz > kSampleRate * 0.45f) cutoff_hz = kSampleRate * 0.45f;

    const float w0     = kTwoPi * cutoff_hz * kInvSR;
    const float cosw0  = std::cos(w0);
    const float sinw0  = std::sin(w0);
    /* Q = 1/sqrt(2) → Butterworth */
    const float Q      = 0.70710678f;
    const float alpha  = sinw0 / (2.0f * Q);

    const float a0     = 1.0f + alpha;
    const float inv_a0 = 1.0f / a0;

    BiquadCoeffs c;
    c.b0 = ((1.0f - cosw0) * 0.5f) * inv_a0;
    c.b1 = ( 1.0f - cosw0)         * inv_a0;
    c.b2 = ((1.0f - cosw0) * 0.5f) * inv_a0;
    c.a1 = (-2.0f * cosw0)         * inv_a0;
    c.a2 = ( 1.0f - alpha)         * inv_a0;
    return c;
}

/* Single-sample biquad TDF-II Transposed update.  z[0], z[1] are the
 * two state variables.                                                */
static inline float BiquadTick(const BiquadCoeffs& c, float z[2], float x)
{
    const float y  = c.b0 * x + z[0];
    z[0] = c.b1 * x - c.a1 * y + z[1];
    z[1] = c.b2 * x - c.a2 * y;
    return y;
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Cached biquad — chord low-pass is static, computed once at Init    */
/* ─────────────────────────────────────────────────────────────────── */

static BiquadCoeffs s_chord_lpf;   /* fi.lowpass(2, 2200)              */

/* Air bandpass: cascade of a 1-pole HP at f_lo and a 1-pole LP at
 * f_hi.  Cheap and audibly indistinguishable from a 2nd-order BPF on
 * pink noise.                                                         */
struct OnePoleAB
{
    float a; /* feed-back / "alpha" pole */
};

static OnePoleAB s_air_hpL, s_air_hpR;   /* high-pass (subtraction)   */
static OnePoleAB s_air_lpL, s_air_lpR;   /* low-pass                  */

static OnePoleAB OnePoleLpDesign(float cutoff_hz)
{
    if (cutoff_hz < 1.0f)               cutoff_hz = 1.0f;
    if (cutoff_hz > kSampleRate * 0.49f) cutoff_hz = kSampleRate * 0.49f;
    const float x = std::exp(-kTwoPi * cutoff_hz * kInvSR);
    return { x };
}

/* y[n] = a*y[n-1] + (1-a)*x[n]                                          */
static inline float OnePoleLpTick(const OnePoleAB& f, float& z, float x)
{
    z = f.a * z + (1.0f - f.a) * x;
    return z;
}

/* High-pass = x − low-pass(x) */
static inline float OnePoleHpTick(const OnePoleAB& f, float& z, float x)
{
    z = f.a * z + (1.0f - f.a) * x;
    return x - z;
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Pink noise (Paul Kellet's "economy" 7-pole filter)                 */
/* ─────────────────────────────────────────────────────────────────── */

float GasChords::NextWhiteNoise()
{
    /* xorshift32 — fast, cheap, plenty random for noise applications. */
    uint32_t x = rng_state_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state_ = x;
    /* Map to [-1, 1) uniformly. */
    return static_cast<float>(static_cast<int32_t>(x)) * (1.0f / 2147483648.0f);
}

float GasChords::NextPinkNoise()
{
    /* Paul Kellet's "economy" pink noise filter.  ~−3 dB/oct from
     * 30 Hz to 14 kHz @ 48 k.                                        */
    const float w = NextWhiteNoise();
    pink_b_[0] = 0.99886f * pink_b_[0] + w * 0.0555179f;
    pink_b_[1] = 0.99332f * pink_b_[1] + w * 0.0750759f;
    pink_b_[2] = 0.96900f * pink_b_[2] + w * 0.1538520f;
    pink_b_[3] = 0.86650f * pink_b_[3] + w * 0.3104856f;
    pink_b_[4] = 0.55000f * pink_b_[4] + w * 0.5329522f;
    pink_b_[5] = -0.7616f * pink_b_[5] - w * 0.0168980f;
    const float pink = pink_b_[0] + pink_b_[1] + pink_b_[2] + pink_b_[3]
                     + pink_b_[4] + pink_b_[5] + pink_b_[6] + w * 0.5362f;
    pink_b_[6] = w * 0.115926f;
    return pink * 0.11f;   /* normalise to roughly ±1                  */
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Lifecycle / setters                                                */
/* ─────────────────────────────────────────────────────────────────── */

void GasChords::Init()
{
    /* Static filter designs. */
    s_chord_lpf = DesignLpfButter(2200.0f);
    s_air_hpL   = OnePoleLpDesign(4000.0f);
    s_air_lpL   = OnePoleLpDesign(8000.0f);
    s_air_hpR   = OnePoleLpDesign(4500.0f);
    s_air_lpR   = OnePoleLpDesign(9000.0f);

    /* Default fog biquad — re-computed every block from smoothed
     * cutoff, but seed it for the first block.                       */
    ComputeFogCoeffs(fog_cutoff_);

    /* Stagger LFO phases so we don't get all-coherent peaks at boot. */
    for (int i = 0; i < 10; i++)
        shmr_lfo_ph_[i] = static_cast<float>(i) * 0.1f;
    sub_lfo_ph_    = 0.0f;

    /* Stagger chord-voice saw phases for a fuller stack at startup. */
    for (int v = 0; v < 4; v++)
    {
        chord_voice_[v].ph[0] = static_cast<float>(v) * 0.07f;
        chord_voice_[v].ph[1] = static_cast<float>(v) * 0.13f + 0.31f;
        chord_voice_[v].ph[2] = static_cast<float>(v) * 0.19f + 0.59f;
    }
    /* Shimmer bell phases. */
    for (int i = 0; i < 4; i++)
        shmr_ph_[i] = static_cast<float>(i) * 0.25f;

    sub_ph_[0] = 0.0f;
    sub_ph_[1] = 0.25f;
    sub_ph_[2] = 0.5f;

    std::memset(chord_lpf_z_, 0, sizeof(chord_lpf_z_));
    std::memset(fog_bq_z_,    0, sizeof(fog_bq_z_));
    std::memset(fog_one_z_,   0, sizeof(fog_one_z_));
    air_hp_z_[0] = air_hp_z_[1] = 0.0f;
    air_lp_z_[0] = air_lp_z_[1] = 0.0f;
    std::memset(pink_b_, 0, sizeof(pink_b_));
}

void GasChords::SetRootMidi(float midi)
{
    if (midi < 24.0f) midi = 24.0f;
    if (midi > 60.0f) midi = 60.0f;
    root_midi_target_ = midi;
}
void GasChords::SetShimmerLevel(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    shmr_level_target_ = v;
}
void GasChords::SetDetuneCents(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 30.0f) v = 30.0f;
    detune_target_ = v;
}
void GasChords::SetMainLevel(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    main_level_target_ = v;
}
void GasChords::SetChordIndex(uint8_t idx)
{
    if (idx >= kNumChords) idx = kNumChords - 1;
    chord_idx_target_ = idx;
}
void GasChords::SetSubLevel(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    sub_level_target_ = v;
}
void GasChords::SetShimmerDrift(float hz)
{
    if (hz < 0.01f) hz = 0.01f;
    if (hz > 2.0f)  hz = 2.0f;
    shmr_drift_target_ = hz;
}
void GasChords::SetShimmerOctave(uint8_t oct)
{
    if (oct > 4) oct = 4;
    shmr_octave_target_ = oct;
}
void GasChords::SetShimmerAir(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    shmr_air_target_ = v;
}
void GasChords::SetFogCutoff(float hz)
{
    if (hz < 200.0f)  hz = 200.0f;
    if (hz > 8000.0f) hz = 8000.0f;
    fog_cutoff_target_ = hz;
}
void GasChords::SetSubWarmth(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    sub_warmth_target_ = v;
}
void GasChords::SetFogModDepth(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    fog_mod_depth_target_ = v;
}

/* Recompute the fog biquad + 1-pole coefficients from a target cutoff. */
void GasChords::ComputeFogCoeffs(float cutoff_hz)
{
    const BiquadCoeffs c = DesignLpfButter(cutoff_hz);
    fog_b0_ = c.b0; fog_b1_ = c.b1; fog_b2_ = c.b2;
    fog_a1_ = c.a1; fog_a2_ = c.a2;

    if (cutoff_hz < 1.0f) cutoff_hz = 1.0f;
    fog_op_a_ = std::exp(-kTwoPi * cutoff_hz * kInvSR);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Audio block                                                        */
/* ─────────────────────────────────────────────────────────────────── */

void GasChords::ProcessBlock(float* out_l, float* out_r, std::size_t n)
{
    /* ── 1. Slew step targets (continuous params).  Discrete params
     *      (chord index, shimmer octave) snap immediately, but the
     *      pitches they affect are slewed via root_midi_/shmr_*.     */
    const float root_midi_t   = root_midi_target_;
    const float shmr_level_t  = shmr_level_target_;
    const float detune_t      = detune_target_;
    const float main_level_t  = main_level_target_;
    const float sub_level_t   = sub_level_target_;
    const float shmr_drift_t  = shmr_drift_target_;
    const float shmr_air_t    = shmr_air_target_;
    const float fog_cutoff_t  = fog_cutoff_target_;
    const float sub_warmth_t  = sub_warmth_target_;
    const float fog_mod_depth_t = fog_mod_depth_target_;

    /* Discrete params jump on block boundaries. */
    chord_idx_   = chord_idx_target_;
    shmr_octave_ = shmr_octave_target_;

    /* ── 2. Resolve chord intervals once per block. */
    const uint8_t cidx = kChordMap[chord_idx_];
    const float   iv3  = static_cast<float>(kIv3[cidx]);
    const float   iv5  = static_cast<float>(kIv5[cidx]);
    const float   iv7  = static_cast<float>(kIv7[cidx]);

    /* ── 3. Advance LFOs once per block (the LFO output is treated as
     *      constant within the 24-sample block, which is far below the
     *      Nyquist of any modulation source we have). */
    const float drift_lfo[4] = {
        SlowLfoTick(&shmr_lfo_ph_[0], shmr_drift_),
        SlowLfoTick(&shmr_lfo_ph_[2], shmr_drift_ * 1.27f),
        SlowLfoTick(&shmr_lfo_ph_[4], shmr_drift_ * 0.83f),
        SlowLfoTick(&shmr_lfo_ph_[6], shmr_drift_ * 1.51f),
    };
    const float breath = 0.5f
        + 0.5f * SlowLfoTick(&shmr_lfo_ph_[8], shmr_drift_ * 0.31f);

    /* Fog cutoff is modulated by the *primary* shimmer-drift LFO
     * (drift_lfo[0] ∈ [-1, +1]) scaled by the user-set depth.  At
     * full depth the cutoff sweeps ±60% around the user's centre
     * frequency; at zero depth the filter is static.  Using the
     * same LFO that drives shimmer detune means the audible
     * filter sweep is locked to the shimmer wobble, and the UI
     * pip on the drift pot is guaranteed to track both.            */
    const float fog_mod = drift_lfo[0] * fog_mod_depth_ * 0.6f;
    float fog_f = fog_cutoff_ * (1.0f + fog_mod);
    if (fog_f < 60.0f)               fog_f = 60.0f;
    if (fog_f > kSampleRate / 2.2f)  fog_f = kSampleRate / 2.2f;
    ComputeFogCoeffs(fog_f);

    /* Sub pulse LFO (one sine, fixed rate). */
    const float sub_lfo_inc = PhaseInc(kSubPulseRate);
    /* (We keep sub_lfo_ph_ stepping per-sample — see below.) */

    /* ── 4. Resolve frequencies once per block.  Saws and sines below
     *      use the per-sample slewed pitches, but recomputing
     *      Midi2Hz() once per block keeps trig out of the inner loop. */

    /* Pre-compute pan gains for the chord voices (static spread = 1.0). */
    float gL[4], gR[4];
    {
        const float p0 = 0.5f - 0.45f * kStereoSpread;
        const float p1 = 0.5f - 0.18f * kStereoSpread;
        const float p2 = 0.5f + 0.18f * kStereoSpread;
        const float p3 = 0.5f + 0.45f * kStereoSpread;
        PanGains(p0, gL[0], gR[0]);
        PanGains(p1, gL[1], gR[1]);
        PanGains(p2, gL[2], gR[2]);
        PanGains(p3, gL[3], gR[3]);
    }

    /* Local copies of smoothed parameters for the inner loop. */
    float root_midi  = root_midi_;
    float shmr_level = shmr_level_;
    float detune     = detune_;
    float main_level = main_level_;
    float sub_level  = sub_level_;
    float shmr_drift = shmr_drift_;
    float shmr_air   = shmr_air_;
    float fog_cutoff = fog_cutoff_;
    float sub_warmth = sub_warmth_;
    float fog_mod_depth = fog_mod_depth_;
    float sub_lfo_ph = sub_lfo_ph_;

    /* ── Pitch → phase-increment conversion happens once per block.
     * Midi2Hz expands an exp2; doing it per sample for ~19 oscillators
     * would burn a large fraction of the CPU on a 48 kHz block.  At
     * 24-sample blocks (≈ 0.5 ms) the resulting "stepped" pitch update
     * is far below any audible threshold and is masked by the
     * downstream low-pass.  The continuous parameter slew still runs
     * per-sample for amplitude / mix / detune cents — those are the
     * parameters that produce zipper noise without it.                */
    const float n0 = root_midi;
    const float n1 = root_midi + iv3;
    const float n2 = root_midi + iv5;
    const float n3 = root_midi + iv7;
    const float d0 = detune;
    const float d1 = detune * 1.13f;
    const float d2 = detune * 0.91f;
    const float d3 = detune * 1.21f;

    const float inc0a = PhaseInc(Midi2Hz(n0 + d0 * 0.01f));
    const float inc0b = PhaseInc(Midi2Hz(n0));
    const float inc0c = PhaseInc(Midi2Hz(n0 - d0 * 0.01f));
    const float inc1a = PhaseInc(Midi2Hz(n1 + d1 * 0.01f));
    const float inc1b = PhaseInc(Midi2Hz(n1));
    const float inc1c = PhaseInc(Midi2Hz(n1 - d1 * 0.01f));
    const float inc2a = PhaseInc(Midi2Hz(n2 + d2 * 0.01f));
    const float inc2b = PhaseInc(Midi2Hz(n2));
    const float inc2c = PhaseInc(Midi2Hz(n2 - d2 * 0.01f));
    const float inc3a = PhaseInc(Midi2Hz(n3 + d3 * 0.01f));
    const float inc3b = PhaseInc(Midi2Hz(n3));
    const float inc3c = PhaseInc(Midi2Hz(n3 - d3 * 0.01f));

    const float sub_note  = root_midi + 12.0f * static_cast<float>(kSubOctave);
    const float sub_fund  = Midi2Hz(sub_note);
    const float sub_inc0  = PhaseInc(sub_fund);
    const float sub_inc1  = PhaseInc(sub_fund * 2.0f);
    const float sub_inc2  = PhaseInc(sub_fund * 3.0f);

    const float shmr_base = root_midi + 12.0f * static_cast<float>(shmr_octave_);
    const float shmr_inc0 = PhaseInc(Midi2Hz(shmr_base)        * (1.0f + 0.001f * drift_lfo[0]));
    const float shmr_inc1 = PhaseInc(Midi2Hz(shmr_base + iv5)  * (1.0f + 0.001f * drift_lfo[1]));
    const float shmr_inc2 = PhaseInc(Midi2Hz(shmr_base + iv3)  * (1.0f + 0.001f * drift_lfo[2]));
    const float shmr_inc3 = PhaseInc(Midi2Hz(shmr_base + iv7)  * (1.0f + 0.001f * drift_lfo[3]));

    /* Local oscillator phases (pulled out so the optimiser can keep
     * them in registers across the loop).                            */
    float v0p0 = chord_voice_[0].ph[0], v0p1 = chord_voice_[0].ph[1], v0p2 = chord_voice_[0].ph[2];
    float v1p0 = chord_voice_[1].ph[0], v1p1 = chord_voice_[1].ph[1], v1p2 = chord_voice_[1].ph[2];
    float v2p0 = chord_voice_[2].ph[0], v2p1 = chord_voice_[2].ph[1], v2p2 = chord_voice_[2].ph[2];
    float v3p0 = chord_voice_[3].ph[0], v3p1 = chord_voice_[3].ph[1], v3p2 = chord_voice_[3].ph[2];

    float shmrp0 = shmr_ph_[0], shmrp1 = shmr_ph_[1];
    float shmrp2 = shmr_ph_[2], shmrp3 = shmr_ph_[3];

    float subp0 = sub_ph_[0], subp1 = sub_ph_[1], subp2 = sub_ph_[2];

    /* Per-block peak accumulators — each band tracks max-abs across
     * the whole block.  Cheap (one cmp + abs per sample); the full
     * envelope-follower decay only runs once at end-of-block.        */
    float main_max = 0.0f;
    float sub_max  = 0.0f;
    float shmr_max = 0.0f;

    /* Per-sample work. */
    for (std::size_t i = 0; i < n; i++)
    {
        /* Slew continuous parameters one sample. */
        root_midi  += (root_midi_t  - root_midi)  * kSmoothSlow;
        shmr_level += (shmr_level_t - shmr_level) * kSmoothFast;
        detune     += (detune_t     - detune)     * kSmoothFast;
        main_level += (main_level_t - main_level) * kSmoothFast;
        sub_level  += (sub_level_t  - sub_level)  * kSmoothFast;
        shmr_drift += (shmr_drift_t - shmr_drift) * kSmoothFast;
        shmr_air   += (shmr_air_t   - shmr_air)   * kSmoothFast;
        fog_cutoff += (fog_cutoff_t - fog_cutoff) * kSmoothFast;
        sub_warmth += (sub_warmth_t - sub_warmth) * kSmoothFast;
        fog_mod_depth += (fog_mod_depth_t - fog_mod_depth) * kSmoothFast;

        /* ── CHORD ──
         * Phase increments resolved at block start (above) so the
         * inner loop avoids the 12 × Midi2Hz exp2 calls per sample. */
        const float v0 = (Saw(v0p0) + Saw(v0p1) + Saw(v0p2)) * (1.0f / 3.0f);
        const float v1 = (Saw(v1p0) + Saw(v1p1) + Saw(v1p2)) * (1.0f / 3.0f);
        const float v2 = (Saw(v2p0) + Saw(v2p1) + Saw(v2p2)) * (1.0f / 3.0f);
        const float v3 = (Saw(v3p0) + Saw(v3p1) + Saw(v3p2)) * (1.0f / 3.0f);

        v0p0 += inc0a; if (v0p0 >= 1.0f) v0p0 -= 1.0f;
        v0p1 += inc0b; if (v0p1 >= 1.0f) v0p1 -= 1.0f;
        v0p2 += inc0c; if (v0p2 >= 1.0f) v0p2 -= 1.0f;
        v1p0 += inc1a; if (v1p0 >= 1.0f) v1p0 -= 1.0f;
        v1p1 += inc1b; if (v1p1 >= 1.0f) v1p1 -= 1.0f;
        v1p2 += inc1c; if (v1p2 >= 1.0f) v1p2 -= 1.0f;
        v2p0 += inc2a; if (v2p0 >= 1.0f) v2p0 -= 1.0f;
        v2p1 += inc2b; if (v2p1 >= 1.0f) v2p1 -= 1.0f;
        v2p2 += inc2c; if (v2p2 >= 1.0f) v2p2 -= 1.0f;
        v3p0 += inc3a; if (v3p0 >= 1.0f) v3p0 -= 1.0f;
        v3p1 += inc3b; if (v3p1 >= 1.0f) v3p1 -= 1.0f;
        v3p2 += inc3c; if (v3p2 >= 1.0f) v3p2 -= 1.0f;

        float chord_l =
              v0 * gL[0] + v1 * gL[1] + v2 * gL[2] + v3 * gL[3];
        float chord_r =
              v0 * gR[0] + v1 * gR[1] + v2 * gR[2] + v3 * gR[3];

        chord_l *= main_level;
        chord_r *= main_level;

        /* Static 2nd-order LP @ 2200 Hz per channel. */
        chord_l = BiquadTick(s_chord_lpf, chord_lpf_z_[0], chord_l);
        chord_r = BiquadTick(s_chord_lpf, chord_lpf_z_[1], chord_r);

        /* Track the chord/main band peak for the UI VU meter. */
        {
            const float al = std::fabs(chord_l);
            const float ar = std::fabs(chord_r);
            const float a  = al > ar ? al : ar;
            if (a > main_max) main_max = a;
        }

        /* ── SUB ── */
        const float subTone =
              SinNorm(subp0)
            + 0.40f * SinNorm(subp1)
            + 0.15f * SinNorm(subp2);
        subp0 += sub_inc0; if (subp0 >= 1.0f) subp0 -= 1.0f;
        subp1 += sub_inc1; if (subp1 >= 1.0f) subp1 -= 1.0f;
        subp2 += sub_inc2; if (subp2 >= 1.0f) subp2 -= 1.0f;

        const float subSat =
            std::tanh(subTone * (1.0f + 4.0f * sub_warmth))
            / (1.0f + 2.0f * sub_warmth);

        /* Pulse LFO (single sine — Faust uses os.osc(subPulse)). */
        const float subEnv =
            (1.0f - kSubPulseDepth)
            + kSubPulseDepth * (0.5f + 0.5f * SinNorm(sub_lfo_ph));
        sub_lfo_ph += sub_lfo_inc; if (sub_lfo_ph >= 1.0f) sub_lfo_ph -= 1.0f;

        const float sub_mono = subSat * subEnv * sub_level;
        {
            const float a = std::fabs(sub_mono);
            if (a > sub_max) sub_max = a;
        }

        /* ── SHIMMER ── */
        const float bellL = 0.5f * (SinNorm(shmrp0) + SinNorm(shmrp1));
        const float bellR = 0.5f * (SinNorm(shmrp2) + SinNorm(shmrp3));

        shmrp0 += shmr_inc0; if (shmrp0 >= 1.0f) shmrp0 -= 1.0f;
        shmrp1 += shmr_inc1; if (shmrp1 >= 1.0f) shmrp1 -= 1.0f;
        shmrp2 += shmr_inc2; if (shmrp2 >= 1.0f) shmrp2 -= 1.0f;
        shmrp3 += shmr_inc3; if (shmrp3 >= 1.0f) shmrp3 -= 1.0f;

        /* Air = bandpass(pink noise) per channel. */
        const float pn_l = NextPinkNoise();
        const float pn_r = NextPinkNoise();
        float airL = OnePoleHpTick(s_air_hpL, air_hp_z_[0], pn_l);
        airL = OnePoleLpTick(s_air_lpL, air_lp_z_[0], airL);
        float airR = OnePoleHpTick(s_air_hpR, air_hp_z_[1], pn_r);
        airR = OnePoleLpTick(s_air_lpR, air_lp_z_[1], airR);
        airL *= shmr_air;
        airR *= shmr_air;

        const float shmr_l_pre = (bellL + airL) * shmr_level * breath;
        const float shmr_r_pre = (bellR + airR) * shmr_level * breath;

        /* Stereo width = 1.0 (static) → width pass-through.            */
        const float shmr_l = shmr_l_pre;
        const float shmr_r = shmr_r_pre;
        {
            const float al = std::fabs(shmr_l);
            const float ar = std::fabs(shmr_r);
            const float a  = al > ar ? al : ar;
            if (a > shmr_max) shmr_max = a;
        }

        /* ── SUM, FOG, MASTER ── */
        float bus_l = chord_l + sub_mono + shmr_l;
        float bus_r = chord_r + sub_mono + shmr_r;

        /* 3rd-order LP: biquad + 1-pole, per channel.  Coefficients
         * cached at block start (see ComputeFogCoeffs above).  We
         * inline the BiquadTick math here to avoid constructing a
         * temporary `BiquadCoeffs` struct each sample.               */
        {
            const float yl = fog_b0_ * bus_l + fog_bq_z_[0][0];
            fog_bq_z_[0][0] = fog_b1_ * bus_l - fog_a1_ * yl + fog_bq_z_[0][1];
            fog_bq_z_[0][1] = fog_b2_ * bus_l - fog_a2_ * yl;
            bus_l = yl;

            const float yr = fog_b0_ * bus_r + fog_bq_z_[1][0];
            fog_bq_z_[1][0] = fog_b1_ * bus_r - fog_a1_ * yr + fog_bq_z_[1][1];
            fog_bq_z_[1][1] = fog_b2_ * bus_r - fog_a2_ * yr;
            bus_r = yr;
        }
        fog_one_z_[0] = fog_op_a_ * fog_one_z_[0]
                      + (1.0f - fog_op_a_) * bus_l;
        fog_one_z_[1] = fog_op_a_ * fog_one_z_[1]
                      + (1.0f - fog_op_a_) * bus_r;
        bus_l = fog_one_z_[0];
        bus_r = fog_one_z_[1];

        /* Master gain + soft saturation. */
        out_l[i] = SoftClip(bus_l * kMasterGain);
        out_r[i] = SoftClip(bus_r * kMasterGain);
    }

    /* ── Persist locals back to members. ── */
    root_midi_  = root_midi;
    shmr_level_ = shmr_level;
    detune_     = detune;
    main_level_ = main_level;
    sub_level_  = sub_level;
    shmr_drift_ = shmr_drift;
    shmr_air_   = shmr_air;
    fog_cutoff_ = fog_cutoff;
    sub_warmth_ = sub_warmth;
    fog_mod_depth_ = fog_mod_depth;
    sub_lfo_ph_ = sub_lfo_ph;

    chord_voice_[0].ph[0] = v0p0; chord_voice_[0].ph[1] = v0p1; chord_voice_[0].ph[2] = v0p2;
    chord_voice_[1].ph[0] = v1p0; chord_voice_[1].ph[1] = v1p1; chord_voice_[1].ph[2] = v1p2;
    chord_voice_[2].ph[0] = v2p0; chord_voice_[2].ph[1] = v2p1; chord_voice_[2].ph[2] = v2p2;
    chord_voice_[3].ph[0] = v3p0; chord_voice_[3].ph[1] = v3p1; chord_voice_[3].ph[2] = v3p2;

    shmr_ph_[0] = shmrp0; shmr_ph_[1] = shmrp1;
    shmr_ph_[2] = shmrp2; shmr_ph_[3] = shmrp3;

    sub_ph_[0] = subp0; sub_ph_[1] = subp1; sub_ph_[2] = subp2;

    /* ── UI-visible meters.  Per-block envelope follower:
     *   peak ← max(new, peak * decay)
     * decay = 0.995 per ~0.5 ms block ⇒ time constant ≈ 100 ms,
     * −60 dB recovery ≈ 280 ms — classic VU response.               */
    constexpr float kMeterDecay = 0.995f;
    const float main_dec = main_peak_ * kMeterDecay;
    const float sub_dec  = sub_peak_  * kMeterDecay;
    const float shmr_dec = shmr_peak_ * kMeterDecay;
    main_peak_  = main_max > main_dec ? main_max : main_dec;
    sub_peak_   = sub_max  > sub_dec  ? sub_max  : sub_dec;
    shmr_peak_  = shmr_max > shmr_dec ? shmr_max : shmr_dec;

    /* Live drift LFO output and post-modulation fog cutoff — these
     * are the *exact* values the audio block just used, so the UI can
     * render pip position and fog ring fill in lockstep with the
     * audible motion (no separate UI-side clock).                    */
    drift_value_     = drift_lfo[0];
    fog_cutoff_out_  = fog_f;
}

} // namespace VoigtpadDsp
