/**
 * @file dsp.h
 * @brief Voigtpad audio engine — hand-port of `docs/gas_chords.dsp`.
 *
 * Always-on stereo drone synth.  Four sound sections (Chord, Sub Drone,
 * Shimmer, Fog) summed and run through a single low-headroom soft-clip
 * master, mirroring the Faust patch:
 *
 *   process = (chordSection, subSection, shmrSection)
 *           : ro.interleave(2, 3) : sum3, sum3
 *           : space (3rd-order LP w/ slow LFO) ;
 *
 * Differences vs. the .dsp file:
 *   • No MIDI: root pitch is exposed as a continuous knob (24..60 MIDI).
 *   • Static params are baked in (per spec):
 *       stereo spread = 1.0,  stereo width = 1.0,
 *       sub pulse rate = 0.37 Hz, sub pulse depth = 0.6,
 *       fog mod rate   = 0.06 Hz, fog mod depth  = 0.4.
 *   • Master gain knob from the patch is replaced with a fixed
 *     attenuation (`kMasterGain`) calibrated so the stereo bus only
 *     reaches the soft-clip onset when all three mix pots
 *     (shimmer/main/sub) are ≥ 0.75.
 *
 * Performance notes:
 *   • All oscillators use phase accumulators + sin LUT (no `sinf` in
 *     the audio inner loop).
 *   • All slow LFOs (sub-Hz to a few Hz) are evaluated once per audio
 *     block (24 samples) and held within the block — inaudible at this
 *     block size.
 *   • Audio-rate knobs (root note, levels, detune, drift, fog cutoff,
 *     warmth, air) are slewed at sample rate via per-sample one-poles
 *     so knob movement is zipper-free.
 *   • No SDRAM allocations needed; all state lives in DTCM/AXI SRAM.
 *   • Audio runs from libDaisy's SAI DMA callback; this engine never
 *     blocks and never allocates.
 */

#ifndef VOIGTPAD_DSP_H
#define VOIGTPAD_DSP_H

#include <cstddef>
#include <cstdint>

namespace VoigtpadDsp {

/* ── Engine-wide constants ───────────────────────────────────────── */

constexpr float kSampleRate = 48000.0f;

/** Block size; matches the audio callback's `size`.  Used only as a
 *  sanity check / for per-block LFO update cadence. */
constexpr std::size_t kBlockSize = 24;

/** Sin LUT power of two; 4096-entry → ~13-bit phase resolution.
 *  Sized in dsp.cpp (kept here as a hint for the smoothing constants). */
constexpr std::size_t kSinTableBits = 12;
constexpr std::size_t kSinTableSize = 1u << kSinTableBits; /* 4096 */
constexpr std::size_t kSinTableMask = kSinTableSize - 1u;

/** Number of voices in the chord. */
constexpr std::size_t kChordVoices = 4;

/** Number of bell partials in the shimmer section. */
constexpr std::size_t kShimmerVoices = 4;

/** Master gain (post-sum, pre soft-clip).  Calibrated so that with
 *  shimmer / main / sub mix all at 0.75, peak loudly hovers below
 *  -6 dBFS — i.e. clipping only kicks in once the user drives any of
 *  the three layer levels above 75%.  See dsp.cpp comment block for
 *  the calibration walk-through.                                       */
constexpr float kMasterGain = 0.45f;

/** Number of chord types selectable via the chord pot. */
constexpr std::size_t kNumChordTypes = 5;

enum class ChordType : uint8_t
{
    Min  = 0,
    Maj  = 1,
    Min7 = 2,
    Sus2 = 3,
    Sus4 = 4,
};

/** Parameter bundle pushed from the UI on every frame.  All values are
 *  in their *natural* engineering units (Hz, MIDI, cents, 0..1) so the
 *  UI layer doesn't have to know about internal smoothing or LUT scales.
 *  The engine handles range-clamping, smoothing and audio-rate updates.  */
struct Params
{
    /* Chord */
    float     root_midi      = 36.0f;     /* 24..60                      */
    float     detune_cents   = 15.0f;     /* 0..30                       */
    ChordType chord_type     = ChordType::Min;
    float     main_level     = 0.5f;      /* 0..1                        */

    /* Shimmer */
    float    shimmer_level   = 0.5f;      /* 0..1                        */
    float    shimmer_drift   = 0.15f;     /* 0.01..1.0 Hz                */
    uint8_t  shimmer_octave  = 2;         /* 0..4                        */
    float    shimmer_air     = 0.5f;      /* 0..1                        */

    /* Sub */
    float    sub_level       = 0.5f;      /* 0..1                        */
    float    sub_warmth      = 0.2f;      /* 0..1                        */

    /* Fog (only base cutoff is user-controlled) */
    float    fog_cutoff_hz   = 4450.0f;   /* 200..8000 Hz                */
};

/* ── Internal sub-sections ───────────────────────────────────────── */

/* Lightweight phase accumulator + table-lookup sin oscillator. */
struct PhaseOsc
{
    uint32_t phase = 0;        /* Q32 phase (full 2π = 2^32) */

    /* Reset phase to a deterministic value (used to stagger LFOs). */
    void Reset(uint32_t p) { phase = p; }
};

/* Single-stage one-pole low-pass (used for warmth-LP and fog stages). */
struct OnePoleLP
{
    float a = 0.0f;   /* coefficient: y += a*(x - y) */
    float z = 0.0f;
    inline float Process(float x)
    {
        z += a * (x - z);
        return z;
    }
};

/* Voss-McCartney pink noise (5 stages — cheap & flat enough for "air"). */
struct PinkNoise
{
    int32_t  rows[5]    = {0, 0, 0, 0, 0};
    int32_t  running    = 0;
    uint32_t counter    = 1;   /* Skipped-update counter (never 0). */
    uint32_t rng_state  = 0xC0FFEEu;

    /* Returns one sample in roughly [-1, 1]. */
    float Process();
};

/* 2nd-order biquad (Direct Form I).  Used for the chord-voice 2.2 kHz
 * lowpass (Faust `fi.lowpass(2, 2200)`) and the shimmer-air bandpass.   */
struct Biquad
{
    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
    float a1 = 0.0f, a2 = 0.0f;
    float x1 = 0.0f, x2 = 0.0f;
    float y1 = 0.0f, y2 = 0.0f;

    inline float Process(float x)
    {
        const float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

/* ── The engine ──────────────────────────────────────────────────── */

class Engine
{
  public:
    /** Initialise all sub-sections, oscillators, smoothers, LUT
     *  references.  Safe to call once at start-up; not thread-safe.    */
    void Init();

    /** Push new control targets.  Safe to call from the UI loop every
     *  frame; the engine slews internally so this is zipper-free.      */
    void SetParams(const Params& p);

    /** Process `n` samples of stereo output.  Inputs are ignored
     *  (drone synth).  `out_l` / `out_r` are written, may alias.
     *  `n` is expected to be `kBlockSize` but the implementation is
     *  not size-locked.                                                 */
    void ProcessBlock(const float* /*in_l*/, const float* /*in_r*/,
                      float* out_l, float* out_r, std::size_t n);

  private:
    /* Smoothed parameter state (target tracked at audio rate). */
    float root_midi_      = 36.0f, root_midi_t_      = 36.0f;
    float detune_cents_   = 15.0f, detune_cents_t_   = 15.0f;
    float main_level_     = 0.5f,  main_level_t_     = 0.5f;
    float shmr_level_     = 0.5f,  shmr_level_t_     = 0.5f;
    float shmr_drift_     = 0.15f, shmr_drift_t_     = 0.15f;
    float shmr_air_       = 0.5f,  shmr_air_t_       = 0.5f;
    float sub_level_      = 0.5f,  sub_level_t_      = 0.5f;
    float sub_warmth_     = 0.2f,  sub_warmth_t_     = 0.2f;
    float fog_cutoff_     = 4450.f,fog_cutoff_t_     = 4450.f;

    /* Discrete params — applied directly (already snapped by UI). */
    ChordType chord_type_   = ChordType::Min;
    uint8_t   shmr_octave_  = 2;

    /* ── Chord section ── */
    PhaseOsc chord_phase_[kChordVoices][3];   /* 3 saws per voice */
    Biquad   chord_lp_l_[kChordVoices];
    Biquad   chord_lp_r_[kChordVoices];

    /* ── Sub section ── */
    PhaseOsc sub_phase_[3];                    /* 1×, 2×, 3× harmonics */
    PhaseOsc sub_pulse_lfo_;                   /* 0.37 Hz, fixed       */

    /* ── Shimmer section ── */
    /* 4 bells + 4 drift LFOs.  Two L bells, two R bells per Faust. */
    PhaseOsc shmr_bell_phase_[kShimmerVoices];
    PhaseOsc shmr_drift_phase_[kShimmerVoices][2]; /* slowLFO = 2 sines */
    PhaseOsc shmr_breath_phase_[2];                /* shared L/R breathing */
    PinkNoise shmr_pink_l_;
    PinkNoise shmr_pink_r_;
    Biquad    shmr_air_bp_l_;
    Biquad    shmr_air_bp_r_;

    /* ── Fog section ── */
    /* Three cascaded 1-pole LPs ≈ 3rd-order Butterworth-ish (matches
     * the Faust `fi.lowpass(3, ...)` topology closely enough for the
     * intended subtle "fog" filtering).                                 */
    OnePoleLP fog_lp_l_[3];
    OnePoleLP fog_lp_r_[3];
    PhaseOsc  fog_lfo_a_;                          /* 0.06 Hz, fixed */
    PhaseOsc  fog_lfo_b_;                          /* 0.06*1.37 Hz   */

    /* Per-block updated (slow) values. */
    float fog_cutoff_eff_ = 4450.0f;

    /* Smoothing coefficient (one-pole α at sample rate, ~25 ms tau).  */
    float smooth_a_ = 0.0f;

    /* Helpers */
    inline void TickSmoothers();
    inline float Tanh(float x) const;
};

} // namespace VoigtpadDsp

#endif /* VOIGTPAD_DSP_H */
