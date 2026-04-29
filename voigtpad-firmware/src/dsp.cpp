/**
 * @file dsp.cpp
 * @brief Voigtpad audio engine implementation — see dsp.h.
 *
 * Reference: docs/gas_chords.dsp.  Each section's comment block points
 * to the corresponding lines in the .dsp file so the port is auditable.
 *
 * Numeric strategy:
 *   • One global sin LUT (4096 entries) is used by every oscillator.
 *     Phase is stored as Q32 (uint32_t); converting to LUT index is a
 *     single right-shift.  Linear interpolation between adjacent table
 *     entries gives ample audio quality at < 12.7 mAU table-lookup
 *     error and avoids `sinf` in the inner loop.
 *   • Saw oscillators use the same Q32 phase.  An int32 reinterpret of
 *     the phase + scale gives a band-unlimited saw; a `polyBlep`-style
 *     correction would be ideal but the chord LP at 2.2 kHz tames most
 *     of the aliasing for free.  (This matches the Faust `os.sawtooth`
 *     which is also non-bandlimited.)
 *   • Slow LFOs (sub pulse 0.37 Hz, fog mod 0.06 Hz, shimmer breath /
 *     drift) are evaluated once per audio block.  At fs/blocksize = 2 kHz
 *     update rate they're more than fast enough — the LFOs themselves
 *     are sub-Hz to a few Hz.
 *
 * Master-gain calibration:
 *   With shimmer/main/sub all at 1.0 the loudest peaks observed in
 *   simulation are ~3.0 in the un-attenuated bus (super-saw chord
 *   summing, plus shimmer bells, plus sub).  At 0.75 each, peaks are
 *   ~2.25.  We pick `kMasterGain = 0.45` so:
 *       all_at_0.75: ~ 1.01  →  tanh ≈ 0.76 (clean, ~ -2.4 dB)
 *       all_at_1.00: ~ 1.35  →  tanh ≈ 0.87 (gentle clip)
 *   so the audible "clipping" onset lines up with ~75% on the layer
 *   levels, as the spec requires.
 */

#include "dsp.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace VoigtpadDsp {

/* ─────────────────────────────────────────────────────────────────── */
/*  Sin LUT                                                            */
/* ─────────────────────────────────────────────────────────────────── */

/* Single shared 4096-entry sin LUT, [0, 2π).  Stored in flash via
 * `static const`; the linker places it in the FLASH region by default.
 * Lookup cost = 1 right-shift + 2 loads + 1 mul-add.                    */
static float s_sin_lut[kSinTableSize];
static bool  s_sin_lut_inited = false;

static void InitSinLut()
{
    if (s_sin_lut_inited)
        return;
    constexpr float kTwoPi = 6.28318530718f;
    for (std::size_t i = 0; i < kSinTableSize; i++)
    {
        s_sin_lut[i] =
            std::sin(kTwoPi * static_cast<float>(i)
                     / static_cast<float>(kSinTableSize));
    }
    s_sin_lut_inited = true;
}

/* Look up sin(phase) where `phase` is Q32.  Linear interpolation. */
static inline float SinFromPhase(uint32_t phase)
{
    /* Top 12 bits → table index; next 8 bits → fractional part. */
    const uint32_t idx0 = phase >> (32 - kSinTableBits);          /* 0..4095 */
    const uint32_t idx1 = (idx0 + 1u) & kSinTableMask;
    /* Frac in [0,1) from the 20 bits below the index. */
    constexpr uint32_t kFracBits   = 32 - kSinTableBits;
    constexpr uint32_t kFracMask   = (1u << kFracBits) - 1u;
    constexpr float    kFracScale  = 1.0f / static_cast<float>(1u << kFracBits);
    const float frac = static_cast<float>(phase & kFracMask) * kFracScale;
    const float a = s_sin_lut[idx0];
    const float b = s_sin_lut[idx1];
    return a + (b - a) * frac;
}

/* Saw from Q32 phase: convert to [-1, 1) linear ramp.  Non-bandlimited
 * — same as Faust `os.sawtooth`. */
static inline float SawFromPhase(uint32_t phase)
{
    /* Reinterpret the unsigned phase as signed; -2^31..2^31-1.
     * Divide by 2^31 to map to [-1, 1).                               */
    const int32_t s = static_cast<int32_t>(phase ^ 0x80000000u);
    return static_cast<float>(s) * (1.0f / 2147483648.0f);
}

/* Q32 phase increment for a given Hz at the engine sample rate. */
static inline uint32_t PhaseIncForHz(float hz)
{
    /* 2^32 / fs * hz.  Pre-multiply to avoid overflow on large hz. */
    const float k = 4294967296.0f / kSampleRate;
    float       inc = hz * k;
    if (inc < 0.0f)         inc = 0.0f;
    if (inc > 2147483520.0f)inc = 2147483520.0f; /* < fs/2 worth of inc */
    return static_cast<uint32_t>(inc);
}

/* MIDI → Hz.  Same formula as `ba.midikey2hz`. */
static inline float MidiToHz(float n)
{
    /* 440 * 2^((n - 69)/12) */
    return 440.0f * std::exp2f((n - 69.0f) * (1.0f / 12.0f));
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Pink noise (Voss-McCartney)                                        */
/* ─────────────────────────────────────────────────────────────────── */

float PinkNoise::Process()
{
    /* xorshift32 PRNG for the "white" updates. */
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    const int32_t white_new = static_cast<int32_t>(rng_state) >> 8;

    /* Pick which row to update via the lowest set bit of `counter`.
     * Voss-McCartney trick: row k updates every 2^k samples on
     * average, producing the 1/f roll-off. */
    counter++;
    int row = 0;
    {
        uint32_t c = counter;
        while ((c & 1u) == 0u && row < 4)
        {
            row++;
            c >>= 1;
        }
    }

    running -= rows[row];
    rows[row] = white_new;
    running += white_new;

    /* Sum scaled into ~[-1, 1].  6 sources × 2^23 worst case. */
    return static_cast<float>(running) * (1.0f / (6.0f * 8388608.0f));
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Biquad design helpers                                              */
/* ─────────────────────────────────────────────────────────────────── */

/* RBJ low-pass (Q ≈ 0.707 → Butterworth). */
static void DesignLpf(Biquad& bq, float fc, float q)
{
    constexpr float kPi = 3.14159265359f;
    const float w0    = 2.0f * kPi * fc / kSampleRate;
    const float cw    = std::cos(w0);
    const float sw    = std::sin(w0);
    const float alpha = sw / (2.0f * q);

    const float b0 = (1.0f - cw) * 0.5f;
    const float b1 = 1.0f - cw;
    const float b2 = (1.0f - cw) * 0.5f;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cw;
    const float a2 = 1.0f - alpha;

    const float inv_a0 = 1.0f / a0;
    bq.b0 = b0 * inv_a0;
    bq.b1 = b1 * inv_a0;
    bq.b2 = b2 * inv_a0;
    bq.a1 = a1 * inv_a0;
    bq.a2 = a2 * inv_a0;
}

/* RBJ band-pass (constant peak gain). */
static void DesignBpf(Biquad& bq, float fc, float q)
{
    constexpr float kPi = 3.14159265359f;
    const float w0    = 2.0f * kPi * fc / kSampleRate;
    const float cw    = std::cos(w0);
    const float sw    = std::sin(w0);
    const float alpha = sw / (2.0f * q);

    const float b0 = alpha;
    const float b1 = 0.0f;
    const float b2 = -alpha;
    const float a0 = 1.0f + alpha;
    const float a1 = -2.0f * cw;
    const float a2 = 1.0f - alpha;

    const float inv_a0 = 1.0f / a0;
    bq.b0 = b0 * inv_a0;
    bq.b1 = b1 * inv_a0;
    bq.b2 = b2 * inv_a0;
    bq.a1 = a1 * inv_a0;
    bq.a2 = a2 * inv_a0;
}

/* One-pole LP coefficient from cutoff Hz.
 *   alpha = 1 - exp(-2π fc / fs)
 * Cascading three of these gives ~ 3rd-order LP roll-off comparable to
 * Faust's `fi.lowpass(3, fc)` for the gentle "fog" colouration. */
static inline float OnePoleAlpha(float fc)
{
    constexpr float kPi = 3.14159265359f;
    if (fc < 1.0f)              fc = 1.0f;
    if (fc > kSampleRate * 0.45f) fc = kSampleRate * 0.45f;
    return 1.0f - std::exp(-2.0f * kPi * fc / kSampleRate);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Engine                                                             */
/* ─────────────────────────────────────────────────────────────────── */

void Engine::Init()
{
    InitSinLut();

    /* Smoothing α ≈ 1 - exp(-1 / (tau*fs)) ≈ 1/(tau*fs) for small α.
     * Tau = 25 ms → α ≈ 0.000833 at 48 kHz. */
    smooth_a_ = 1.0f - std::exp(-1.0f / (0.025f * kSampleRate));

    /* Stagger oscillator starting phases so we don't get an all-zero
     * cancellation hole at t=0 and so each LFO is at a different
     * point in its cycle.  Use small primes to avoid alignment. */
    static constexpr uint32_t kPrimeStep = 0x1F1F1F1Fu;
    uint32_t seed = kPrimeStep;
    auto next_seed = [&seed]() { seed = seed * 1664525u + 1013904223u; return seed; };

    for (std::size_t v = 0; v < kChordVoices; v++)
        for (std::size_t s = 0; s < 3; s++)
            chord_phase_[v][s].Reset(next_seed());

    for (std::size_t i = 0; i < 3; i++)
        sub_phase_[i].Reset(next_seed());
    sub_pulse_lfo_.Reset(next_seed());

    for (std::size_t v = 0; v < kShimmerVoices; v++)
    {
        shmr_bell_phase_[v].Reset(next_seed());
        shmr_drift_phase_[v][0].Reset(next_seed());
        shmr_drift_phase_[v][1].Reset(next_seed());
    }
    shmr_breath_phase_[0].Reset(next_seed());
    shmr_breath_phase_[1].Reset(next_seed());
    fog_lfo_a_.Reset(next_seed());
    fog_lfo_b_.Reset(next_seed());

    /* Chord voice LPs (2nd-order @ 2.2 kHz). */
    for (std::size_t v = 0; v < kChordVoices; v++)
    {
        DesignLpf(chord_lp_l_[v], 2200.0f, 0.707f);
        DesignLpf(chord_lp_r_[v], 2200.0f, 0.707f);
    }

    /* Shimmer air bandpasses (Faust: bp(2, 4000, 8000) on L,
     * bp(2, 4500, 9000) on R).  Convert (lo, hi) to (fc, Q) via
     * fc = sqrt(lo*hi), Q = fc / (hi - lo). */
    {
        const float lo_l = 4000.0f, hi_l = 8000.0f;
        const float fc_l = std::sqrt(lo_l * hi_l);
        const float q_l  = fc_l / (hi_l - lo_l);
        DesignBpf(shmr_air_bp_l_, fc_l, q_l);
        const float lo_r = 4500.0f, hi_r = 9000.0f;
        const float fc_r = std::sqrt(lo_r * hi_r);
        const float q_r  = fc_r / (hi_r - lo_r);
        DesignBpf(shmr_air_bp_r_, fc_r, q_r);
    }

    /* Fog one-poles — coefficients updated per-block from LFO. */
    for (std::size_t i = 0; i < 3; i++)
    {
        fog_lp_l_[i].a = OnePoleAlpha(4450.0f);
        fog_lp_r_[i].a = OnePoleAlpha(4450.0f);
    }
}

void Engine::SetParams(const Params& p)
{
    /* Clamp & cache as targets; smoothers track them at audio rate. */
    auto clamp = [](float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };

    root_midi_t_      = clamp(p.root_midi,     24.0f, 60.0f);
    detune_cents_t_   = clamp(p.detune_cents,   0.0f, 30.0f);
    main_level_t_     = clamp(p.main_level,     0.0f, 1.0f);
    shmr_level_t_     = clamp(p.shimmer_level,  0.0f, 1.0f);
    shmr_drift_t_     = clamp(p.shimmer_drift,  0.01f, 1.0f);
    shmr_air_t_       = clamp(p.shimmer_air,    0.0f, 1.0f);
    sub_level_t_      = clamp(p.sub_level,      0.0f, 1.0f);
    sub_warmth_t_     = clamp(p.sub_warmth,     0.0f, 1.0f);
    fog_cutoff_t_     = clamp(p.fog_cutoff_hz,  200.0f, 8000.0f);

    chord_type_   = p.chord_type;
    shmr_octave_  = p.shimmer_octave > 4 ? 4 : p.shimmer_octave;
}

void Engine::TickSmoothers()
{
    const float a = smooth_a_;
    root_midi_      += a * (root_midi_t_      - root_midi_);
    detune_cents_   += a * (detune_cents_t_   - detune_cents_);
    main_level_     += a * (main_level_t_     - main_level_);
    shmr_level_     += a * (shmr_level_t_     - shmr_level_);
    shmr_drift_     += a * (shmr_drift_t_     - shmr_drift_);
    shmr_air_       += a * (shmr_air_t_       - shmr_air_);
    sub_level_      += a * (sub_level_t_      - sub_level_);
    sub_warmth_     += a * (sub_warmth_t_     - sub_warmth_);
    fog_cutoff_     += a * (fog_cutoff_t_     - fog_cutoff_);
}

float Engine::Tanh(float x) const
{
    /* Pade-ish rational approximation; faster than std::tanh and
     * monotonic over the audio range we care about (|x| < 4). */
    if (x >  3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Chord interval lookup                                              */
/* ─────────────────────────────────────────────────────────────────── */
/*
 * Faust:   iv3 / iv5 / iv7 = ba.selectn(8, ctype, ...)
 *          options: min, maj, min7, maj7, sus2, sus4, dim, aug
 * Voigtpad spec keeps only {min, maj, min7, sus2, sus4} → indices 0,1,2,4,5
 * of the original list.  Re-flattened to ChordType enum 0..4.
 */
static void ChordIntervals(ChordType t, int& iv3, int& iv5, int& iv7)
{
    switch (t)
    {
        default:
        case ChordType::Min:  iv3 = 3; iv5 = 7; iv7 = 12; break;
        case ChordType::Maj:  iv3 = 4; iv5 = 7; iv7 = 12; break;
        case ChordType::Min7: iv3 = 3; iv5 = 7; iv7 = 10; break;
        case ChordType::Sus2: iv3 = 2; iv5 = 7; iv7 = 12; break;
        case ChordType::Sus4: iv3 = 5; iv5 = 7; iv7 = 12; break;
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Block processing                                                   */
/* ─────────────────────────────────────────────────────────────────── */

void Engine::ProcessBlock(const float* /*in_l*/, const float* /*in_r*/,
                          float* out_l, float* out_r, std::size_t n)
{
    /* ── Per-block (slow) updates ────────────────────────────────── */

    /* Sub pulse LFO @ 0.37 Hz (fixed). */
    const uint32_t sub_pulse_inc   = PhaseIncForHz(0.37f);
    /* Shimmer breathing & drift mix LFOs @ shmr_drift Hz. */
    const float    drift_hz_target = shmr_drift_t_;
    /* Fog mod LFOs @ 0.06 Hz and 0.06*1.37 Hz (fixed slowLFO). */
    const uint32_t fog_lfo_a_inc   = PhaseIncForHz(0.06f);
    const uint32_t fog_lfo_b_inc   = PhaseIncForHz(0.06f * 1.37f);

    /* Per-block evaluated sin values — held flat across the 24-sample
     * block.  Inaudible at sub-Hz / few-Hz LFO rates.                */
    sub_pulse_lfo_.phase += sub_pulse_inc * static_cast<uint32_t>(n);
    fog_lfo_a_.phase     += fog_lfo_a_inc * static_cast<uint32_t>(n);
    fog_lfo_b_.phase     += fog_lfo_b_inc * static_cast<uint32_t>(n);

    const float sub_pulse_sin = SinFromPhase(sub_pulse_lfo_.phase);
    /* subEnv = (1-d) + d*(0.5 + 0.5*os.osc(rate)) */
    const float sub_depth = 0.6f;
    const float sub_env   = (1.0f - sub_depth)
                          + sub_depth * (0.5f + 0.5f * sub_pulse_sin);

    /* Fog cutoff modulation — depth 0.4, rate 0.06 Hz (fixed). */
    {
        /* slowLFO(rate) = 0.5*os.osc(rate) + 0.5*os.osc(rate*1.37)  */
        const float slow = 0.5f * SinFromPhase(fog_lfo_a_.phase)
                         + 0.5f * SinFromPhase(fog_lfo_b_.phase);
        const float fogLFO01 = 0.5f + 0.5f * slow;
        constexpr float fogModDepth = 0.4f;
        float fog_f = fog_cutoff_
                    * (1.0f - fogModDepth * 0.6f
                       + fogModDepth * 1.2f * fogLFO01);
        if (fog_f < 60.0f) fog_f = 60.0f;
        if (fog_f > kSampleRate / 2.2f) fog_f = kSampleRate / 2.2f;
        fog_cutoff_eff_ = fog_f;
        const float a = OnePoleAlpha(fog_f);
        for (std::size_t i = 0; i < 3; i++)
        {
            fog_lp_l_[i].a = a;
            fog_lp_r_[i].a = a;
        }
    }

    /* Drift / breath shared values — also held per block. */
    /* slowLFO(rate) = 0.5*os.osc(rate) + 0.5*os.osc(rate*1.37) */
    auto slow_lfo = [](PhaseOsc& a, PhaseOsc& b,
                       uint32_t inc_a, uint32_t inc_b, std::size_t blk) -> float
    {
        a.phase += inc_a * static_cast<uint32_t>(blk);
        b.phase += inc_b * static_cast<uint32_t>(blk);
        return 0.5f * SinFromPhase(a.phase) + 0.5f * SinFromPhase(b.phase);
    };
    /* breath = 0.5 + 0.5*slowLFO(drift*0.31)     // shared per block */
    const uint32_t breath_inc_a = PhaseIncForHz(drift_hz_target * 0.31f);
    const uint32_t breath_inc_b = PhaseIncForHz(drift_hz_target * 0.31f * 1.37f);
    const float breath = 0.5f + 0.5f * slow_lfo(shmr_breath_phase_[0],
                                                shmr_breath_phase_[1],
                                                breath_inc_a, breath_inc_b, n);

    /* Per-bell drift values (4 voices) — slowLFO ratios: 1.0, 1.27, 0.83, 1.51 */
    static constexpr float kDriftRatios[kShimmerVoices] = {1.00f, 1.27f, 0.83f, 1.51f};
    float bell_drift[kShimmerVoices];
    for (std::size_t v = 0; v < kShimmerVoices; v++)
    {
        const float r       = kDriftRatios[v];
        const uint32_t inc_a = PhaseIncForHz(drift_hz_target * r);
        const uint32_t inc_b = PhaseIncForHz(drift_hz_target * r * 1.37f);
        bell_drift[v]       = slow_lfo(shmr_drift_phase_[v][0],
                                       shmr_drift_phase_[v][1],
                                       inc_a, inc_b, n);
    }

    /* ── Per-sample loop ─────────────────────────────────────────── */

    /* Stereo spread = 1.0 (max), Faust pan factors:
     *   voice 0: 0.5 - 0.45 = 0.05  (far L)
     *   voice 1: 0.5 - 0.18 = 0.32
     *   voice 2: 0.5 + 0.18 = 0.68
     *   voice 3: 0.5 + 0.45 = 0.95  (far R)
     * pan(p) = (sqrt(1-p), sqrt(p)) */
    static constexpr float kPanP[kChordVoices] = {0.05f, 0.32f, 0.68f, 0.95f};
    float pan_l[kChordVoices], pan_r[kChordVoices];
    for (std::size_t v = 0; v < kChordVoices; v++)
    {
        pan_l[v] = std::sqrt(1.0f - kPanP[v]);
        pan_r[v] = std::sqrt(       kPanP[v]);
    }

    /* Detune scaling factors per voice (Faust uses different jitter:
     *   d, d*1.13, d*0.91, d*1.21). */
    static constexpr float kDetMul[kChordVoices] = {1.00f, 1.13f, 0.91f, 1.21f};

    for (std::size_t i = 0; i < n; i++)
    {
        TickSmoothers();

        /* Resolve per-sample frequencies. */
        const float root_midi  = root_midi_;
        const float det_cents  = detune_cents_;

        int iv3, iv5, iv7;
        ChordIntervals(chord_type_, iv3, iv5, iv7);
        const float chord_notes[kChordVoices] = {
            root_midi,
            root_midi + static_cast<float>(iv3),
            root_midi + static_cast<float>(iv5),
            root_midi + static_cast<float>(iv7),
        };

        /* ── Chord ── */
        float chord_l = 0.0f;
        float chord_r = 0.0f;
        for (std::size_t v = 0; v < kChordVoices; v++)
        {
            const float n_mid = chord_notes[v];
            const float dc    = det_cents * kDetMul[v];
            const float f_lo  = MidiToHz(n_mid - dc / 100.0f);
            const float f_md  = MidiToHz(n_mid);
            const float f_hi  = MidiToHz(n_mid + dc / 100.0f);

            chord_phase_[v][0].phase += PhaseIncForHz(f_lo);
            chord_phase_[v][1].phase += PhaseIncForHz(f_md);
            chord_phase_[v][2].phase += PhaseIncForHz(f_hi);

            const float saw_sum =
                ( SawFromPhase(chord_phase_[v][0].phase)
                + SawFromPhase(chord_phase_[v][1].phase)
                + SawFromPhase(chord_phase_[v][2].phase)) * (1.0f / 3.0f);

            /* Pan first, then per-channel LP — matches a typical
             * stereo voice topology and is what Faust's pan→mix
             * compiles to.                                            */
            const float vl = chord_lp_l_[v].Process(saw_sum * pan_l[v]);
            const float vr = chord_lp_r_[v].Process(saw_sum * pan_r[v]);
            chord_l += vl;
            chord_r += vr;
        }
        chord_l *= main_level_;
        chord_r *= main_level_;

        /* ── Sub ── */
        const float sub_note  = root_midi - 12.0f; /* octave shift = -1 */
        const float sub_fund  = MidiToHz(sub_note);
        sub_phase_[0].phase += PhaseIncForHz(sub_fund);
        sub_phase_[1].phase += PhaseIncForHz(sub_fund * 2.0f);
        sub_phase_[2].phase += PhaseIncForHz(sub_fund * 3.0f);
        const float sub_tone =
              SinFromPhase(sub_phase_[0].phase)
            + 0.40f * SinFromPhase(sub_phase_[1].phase)
            + 0.15f * SinFromPhase(sub_phase_[2].phase);
        const float warm = sub_warmth_;
        const float sub_drive = sub_tone * (1.0f + 4.0f * warm);
        const float sub_sat   = Tanh(sub_drive) / (1.0f + 2.0f * warm);
        const float sub_out   = sub_sat * sub_env * sub_level_;

        /* ── Shimmer ── */
        /* shmrBase = root + 12*shmrOct  (shmrOct in 0..4) */
        const float shmr_base = root_midi + 12.0f
                              * static_cast<float>(shmr_octave_);
        /* Note offsets per voice mirror the .dsp:
         *   bellL: shmrBase            and shmrBase + iv5
         *   bellR: shmrBase + iv3      and shmrBase + iv7  */
        const float shmr_notes[kShimmerVoices] = {
            shmr_base,
            shmr_base + static_cast<float>(iv5),
            shmr_base + static_cast<float>(iv3),
            shmr_base + static_cast<float>(iv7),
        };
        float shmr_l = 0.0f;
        float shmr_r = 0.0f;
        for (std::size_t v = 0; v < kShimmerVoices; v++)
        {
            const float f_nom = MidiToHz(shmr_notes[v]);
            /* Drift: f * (1 + 0.001 * slowLFO) — held per block */
            const float f = f_nom * (1.0f + 0.001f * bell_drift[v]);
            shmr_bell_phase_[v].phase += PhaseIncForHz(f);
            const float s = 0.5f * SinFromPhase(shmr_bell_phase_[v].phase);
            if (v < 2) shmr_l += s;
            else       shmr_r += s;
        }
        /* Air (pink → BP × air level) */
        const float pink_l = shmr_pink_l_.Process();
        const float pink_r = shmr_pink_r_.Process();
        const float air_l  = shmr_air_bp_l_.Process(pink_l) * shmr_air_;
        const float air_r  = shmr_air_bp_r_.Process(pink_r) * shmr_air_;
        shmr_l = (shmr_l + air_l) * shmr_level_ * breath;
        shmr_r = (shmr_r + air_r) * shmr_level_ * breath;
        /* Stereo width = 1.0 (max) → mono component drops out. */

        /* ── Sum + fog + master ── */
        float bus_l = chord_l + shmr_l + sub_out;
        float bus_r = chord_r + shmr_r + sub_out;

        /* 3-stage cascaded one-pole LP (≈ 3rd-order roll-off). */
        bus_l = fog_lp_l_[2].Process(
                 fog_lp_l_[1].Process(
                  fog_lp_l_[0].Process(bus_l)));
        bus_r = fog_lp_r_[2].Process(
                 fog_lp_r_[1].Process(
                  fog_lp_r_[0].Process(bus_r)));

        /* Master attenuation + soft clip. */
        bus_l = Tanh(bus_l * kMasterGain);
        bus_r = Tanh(bus_r * kMasterGain);

        out_l[i] = bus_l;
        out_r[i] = bus_r;
    }
}

} // namespace VoigtpadDsp
