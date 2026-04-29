/**
 * @file dsp.h
 * @brief Voigtpad audio engine — hand-translated `gas_chords.dsp` Faust patch.
 *
 * The engine is an always-droning, four-column ambient texture generator
 * adapted from Wolfgang Voigt-style GAS textures:
 *
 *   1. CHORD       — 4-voice superSaw chord, low-passed and panned across
 *                    a stereo field at constant maximum spread.
 *   2. SUB DRONE   — Sine fundamental + 2 harmonics, tanh-saturated and
 *                    slow-pulsed by a low-frequency LFO.
 *   3. SHIMMER     — Two pairs of slowly-detuned sine bells (one pair per
 *                    channel), plus per-channel band-passed pink noise
 *                    "air".  Slow drift LFO modulates the bell pitches;
 *                    a slow breath LFO modulates overall amplitude.
 *                    Stereo width hard-coded to 1.0.
 *   4. FOG         — Stereo 3rd-order low-pass whose cutoff is slowly
 *                    modulated by a long-period LFO. Acts on the summed
 *                    L/R bus before the master gain.
 *
 * All control parameters are smoothed at audio rate (one-pole) to keep
 * the engine click- and zipper-noise free during knob movement, just
 * like the V1 athanor delay.  Static parameters (`spread`, `width`,
 * sub pulse rate / depth, fog mod rate / depth) are baked in as
 * compile-time constants per the spec.
 *
 * Topology (per sample):
 *
 *   ┌─ chordSection ─┐
 *   │                ├──┐
 *   ├─ subSection ───┤  ├─► fog (3rd-order LP, slow-mod cutoff) ─► master
 *   │                ├──┘                                          gain
 *   └─ shmrSection ──┘
 */

#ifndef DSP_H
#define DSP_H

#include <cstddef>
#include <cstdint>

namespace VoigtpadDsp {

/* ── Engine-wide constants ───────────────────────────────────────── */

/** Audio sample rate.  Mirrors hardware.h's ENGINE_SAMPLE_RATE_HZ. */
constexpr float kSampleRate = 48000.0f;

/* ── Static (non-user-exposed) parameters ────────────────────────── */
/*
 * Per spec, these are fixed at compile time.  See gas_chords.dsp for
 * their original UI ranges.
 */
constexpr float kStereoSpread  = 1.0f;
constexpr float kStereoWidth   = 1.0f;
constexpr float kSubPulseRate  = 0.37f;   /* Hz */
constexpr float kSubPulseDepth = 0.60f;
constexpr float kFogModDepth   = 0.40f;
constexpr float kFogModRate    = 0.06f;   /* Hz */

/* ── Chord taxonomy (5 options as per spec) ──────────────────────── */
/*
 * Indices in [0, 4]:  0=min, 1=maj, 2=min7, 3=sus2, 4=sus4
 *
 * Each chord is built as (root, root + iv3, root + iv5, root + iv7) MIDI
 * notes per the original Faust patch.  iv7 = 12 means "octave above
 * the root", which is the Faust default for sus2/sus4/min/maj.
 */
constexpr uint8_t kNumChords = 5;

/* ── Chord-voice oscillator state (3 saws per superSaw) ─────────── */
struct SuperSawState
{
    float ph[3] = {0.0f, 0.0f, 0.0f}; /* phase 0..1 per saw */
};

/* ── Audio engine ────────────────────────────────────────────────── */

/**
 * GasChords — the synthesis engine.  Stereo output, no audio input
 * (drone-only).  All public setters are safe to call from the UI loop;
 * targets are slewed at audio rate.
 */
class GasChords
{
  public:
    /** Initialise oscillator state, parameter targets, smoothers. */
    void Init();

    /* ── Page-1 setters ─────────────────────────────────────────── */
    void SetRootMidi(float midi);                /* 24..60 */
    void SetShimmerLevel(float v);               /* 0..1   */
    void SetDetuneCents(float v);                /* 0..30  */
    void SetMainLevel(float v);                  /* 0..1   */
    void SetChordIndex(uint8_t idx);             /* 0..4   */
    void SetSubLevel(float v);                   /* 0..1   */

    /* ── Page-2 setters ─────────────────────────────────────────── */
    void SetShimmerDrift(float hz);              /* 0.01..1.0 */
    void SetShimmerOctave(uint8_t oct);          /* 0..4 (octaves above root) */
    void SetShimmerAir(float v);                 /* 0..1 */
    void SetFogCutoff(float hz);                 /* 200..8000 */
    void SetSubWarmth(float v);                  /* 0..1 */

    /** Render one block of stereo samples. `out_l` and `out_r` may
     *  alias.  No audio input is consumed — engine drones. */
    void ProcessBlock(float* out_l, float* out_r, std::size_t n);

    /* ── Read-only metering for the UI ────────────────────────────
     * Peak-following per-band level meters, updated once per audio
     * block.  All three include their respective level pot, so the
     * value is the "audible loudness" of that band roughly in the
     * 0..1+ range (post-master headroom).  These are written by the
     * audio thread and read by the UI thread; reads of an aligned
     * 32-bit float are tear-free on the M7 so plain floats are safe.
     *
     * Decay constant is set so a transient takes ≈ 200 ms to drop
     * by 60 dB — classic VU response.                                */
    float MainPeak()    const { return main_peak_; }
    float SubPeak()     const { return sub_peak_;  }
    float ShimmerPeak() const { return shmr_peak_; }

    /** Phase (0..1) of the primary shimmer-drift LFO, used by the UI
     *  to draw a "rotating pip" visualisation that is genuinely in
     *  sync with the audio LFO rather than running its own clock. */
    float DriftPhase()  const { return drift_phase_; }

  private:
    /* ───────── parameter targets (UI-thread side) ───────── */
    float   root_midi_target_     = 36.0f;
    float   shmr_level_target_    = 0.50f;
    float   detune_target_        = 15.0f;     /* cents */
    float   main_level_target_    = 0.50f;
    uint8_t chord_idx_target_     = 0;         /* min */
    float   sub_level_target_     = 0.50f;
    float   shmr_drift_target_    = 0.15f;     /* Hz */
    uint8_t shmr_octave_target_   = 2;
    float   shmr_air_target_      = 0.50f;
    float   fog_cutoff_target_    = 4450.0f;
    float   sub_warmth_target_    = 0.20f;

    /* ───────── audio-rate smoothed parameters ───────── */
    float   root_midi_   = 36.0f;
    float   shmr_level_  = 0.50f;
    float   detune_      = 15.0f;
    float   main_level_  = 0.50f;
    uint8_t chord_idx_   = 0;
    float   sub_level_   = 0.50f;
    float   shmr_drift_  = 0.15f;
    uint8_t shmr_octave_ = 2;
    float   shmr_air_    = 0.50f;
    float   fog_cutoff_  = 4450.0f;
    float   sub_warmth_  = 0.20f;

    /* ───────── oscillator phases (radian-free, 0..1 normalised) ───
     * Audio-rate sine/saw oscillators are run from a 0..1 phase
     * accumulator and converted with a tiny polynomial / lookup as
     * needed.                                                       */
    SuperSawState chord_voice_[4];

    float sub_ph_[3]  = {0.0f, 0.0f, 0.0f};      /* fundamental, *2, *3 */
    float sub_lfo_ph_ = 0.0f;                    /* sub pulse LFO       */

    /* Shimmer bell oscillator phases (4 sines, one per partial). */
    float shmr_ph_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* Shimmer drift slowLFO phases.  slowLFO() = 0.5*sin(2π·rate·t)
     * + 0.5*sin(2π·rate·1.37·t), so 2 sub-oscillators per slowLFO
     * and we have 5 slow-LFOs (4 drift + 1 breath) → 10 phases. */
    float shmr_lfo_ph_[10] = {0.0f};

    /* Fog cutoff slowLFO phases (1 slowLFO, 2 sub-oscillators). */
    float fog_lfo_ph_[2] = {0.0f, 0.0f};

    /* ───────── filters ───────── */

    /* Chord low-pass: fi.lowpass(2, 2200) — 2nd-order Butterworth, one
     * biquad.  Coefficients computed once; static cutoff. Stereo, so
     * one biquad per channel.                                       */
    float chord_lpf_z_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

    /* Air bandpass per channel (fi.bandpass(2, f1, f2)) — implemented
     * as a one-pole HP cascaded with a one-pole LP per channel.     */
    float air_hp_z_[2] = {0.0f, 0.0f};
    float air_lp_z_[2] = {0.0f, 0.0f};

    /* Fog low-pass (3rd-order): biquad cascaded with a one-pole.    */
    float fog_bq_z_[2][2]  = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    float fog_one_z_[2]    = {0.0f, 0.0f};
    /* Cached biquad coefficients (re-computed once per block from
     * the smoothed cutoff so we don't pay the cosf/sinf per sample). */
    float fog_b0_ = 0.0f, fog_b1_ = 0.0f, fog_b2_ = 0.0f;
    float fog_a1_ = 0.0f, fog_a2_ = 0.0f;
    float fog_op_a_ = 0.0f; /* one-pole alpha cached per block       */

    /* Pink noise generator (Paul Kellet's "economy" filter). */
    float pink_b_[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t rng_state_ = 0xA5A5A5A5u;

    /* ── UI-visible meters (written by audio thread, read by UI). ─
     * Plain floats — single-writer / single-reader, aligned 32-bit
     * accesses are tear-free on the M7.                              */
    float main_peak_   = 0.0f;
    float sub_peak_    = 0.0f;
    float shmr_peak_   = 0.0f;
    float drift_phase_ = 0.0f;

    /* ───────── helpers ───────── */
    void  ComputeFogCoeffs(float cutoff_hz);
    float NextWhiteNoise();
    float NextPinkNoise();
};

} // namespace VoigtpadDsp

#endif /* DSP_H */
