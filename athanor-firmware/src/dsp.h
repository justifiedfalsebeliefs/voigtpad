/**
 * @file dsp.h
 * @brief V1 audio DSP for the Athanor dual-delay module.
 *
 * Each delay line is a multi-tap delay buffer feeding a Schroeder
 * all-pass diffusion network with a damped feedback loop:
 *
 *     in ──┬─────────────────────────────────────────────┐
 *          │                                              │
 *          ▼                                              │
 *      [delay buf] ──► [multi-tap mixer] ──► [diffuser] ──┴─► out
 *          ▲                                              │
 *          │                                              │
 *          └─── [feedback × damping low-pass] ◄───────────┘
 *
 * The two delay lines are stacked in series stereo (V1 routing):
 *
 *     in_L ─► Delay1_L ─► Delay2_L ─► out_L
 *     in_R ─► Delay1_R ─► Delay2_R ─► out_R
 *
 * All delay buffers live in external SDRAM (DSY_SDRAM_BSS).
 *
 * Public knobs per delay line:
 *   • delay time    1 ms .. 2 s   (logarithmic taper applied at UI layer)
 *   • feedback      0 .. 1.10     (slightly past unity, stabilised)
 *   • mix           0 .. 1        (dry/wet)
 *   • diffusion     0 .. 1        (all-pass g coefficient: 0 = transparent
 *                                  discrete echoes, 1 = max smear / wash)
 *   • damping       0 .. 1        (one-pole LP cutoff inside the feedback
 *                                  loop: 0 = full bandwidth, 1 = mossy)
 *   • bloom         0 .. 1        (size / movement of the diffuser network:
 *                                  0 = short fixed all-pass loops (clean
 *                                  delay), 1 = long, slowly-modulated
 *                                  loops that smear the line into a
 *                                  full plate/hall tail.  At bloom ≈ 0
 *                                  the LFOs and fractional-read path are
 *                                  bypassed so the cost is the same as
 *                                  a static diffuser.)
 *
 * All control parameters are smoothed at audio rate (one-pole) to keep
 * the engine click- and zipper-noise free during knob movement.
 */

#ifndef DSP_H
#define DSP_H

#include <cstddef>
#include <cstdint>

namespace AthanorDsp {

/* ── Engine-wide constants ───────────────────────────────────────── */

/** Audio sample rate.  Mirrors hardware.h's ENGINE_SAMPLE_RATE_HZ. */
constexpr float kSampleRate = 48000.0f;

/** Maximum addressable delay length in seconds.
 *  A bit above the 2 s spec to give the smoothing/interp some headroom. */
constexpr float kMaxDelaySeconds = 2.05f;

/** Minimum addressable delay length in seconds. */
constexpr float kMinDelaySeconds = 0.001f;

/** Delay-buffer size in samples — power of two so we can mask the
 *  read/write index instead of using a modulo.  131 072 ≈ 2.73 s @ 48 kHz,
 *  so 2 s + interpolation headroom always fits. */
constexpr std::size_t kDelayBufSize = 131072u;
constexpr std::size_t kDelayBufMask = kDelayBufSize - 1u;
static_assert((kDelayBufSize & kDelayBufMask) == 0,
              "kDelayBufSize must be a power of two");

/** Number of Schroeder all-pass diffusion stages per delay line. */
constexpr std::size_t kNumAllpasses = 4;

/** Number of delay-buffer read taps per line. */
constexpr std::size_t kNumTaps = 4;

/** Per-stage Schroeder all-pass loop lengths (samples).
 *
 * The diffuser is "morphable" — at bloom = 0 the active loop length is
 * `kApMin*` (short, hand-picked mutually-prime values in the 5–15 ms
 * range, identical to the V1 fixed-length diffuser); at bloom = 1 it
 * extends to `kApMax*` (40–120 ms range), with a slow per-stage LFO
 * jittering the length by a few percent on top.  The backing SDRAM
 * buffers are sized to the maxima so the active length can sweep
 * continuously without reallocation.  Lengths are mutually prime so
 * the modes don't pile up at the same frequencies.                   */
constexpr std::size_t kApMin0 = 173;
constexpr std::size_t kApMin1 = 269;
constexpr std::size_t kApMin2 = 379;
constexpr std::size_t kApMin3 = 521;
constexpr std::size_t kApMax0 = 1901;   /* ≈  39.6 ms @ 48 kHz */
constexpr std::size_t kApMax1 = 2953;   /* ≈  61.5 ms @ 48 kHz */
constexpr std::size_t kApMax2 = 4153;   /* ≈  86.5 ms @ 48 kHz */
constexpr std::size_t kApMax3 = 5749;   /* ≈ 119.8 ms @ 48 kHz */

/* ── Per-line delay engine ───────────────────────────────────────── */

class DelayLine
{
  public:
    /** Bind the engine to its externally-allocated SDRAM ring buffer
     *  and the four diffuser buffers.  `lfo_phase_offset` is added to
     *  every per-stage LFO so each (line, channel) pair runs at a
     *  different point in its slow sweep — gives natural stereo
     *  motion when bloom > 0.  Must be called once at start-up.       */
    void Init(float* main_buf,
              float* ap_buf0, std::size_t ap_buf_len0,
              float* ap_buf1, std::size_t ap_buf_len1,
              float* ap_buf2, std::size_t ap_buf_len2,
              float* ap_buf3, std::size_t ap_buf_len3,
              float  lfo_phase_offset);

    /** Set raw control targets.  Values are slewed internally so it is
     *  safe to call this from the audio thread every block.            */
    void SetParams(float delay_seconds, float feedback, float mix,
                   float diffusion,     float damping,  float bloom);

    /** Process a block of `n` samples in-place / sample-by-sample.
     *  `dry_in` is the wet input (already includes any upstream sound),
     *  result is mixed dry/wet according to the mix parameter.          */
    void ProcessBlock(const float* in, float* out, std::size_t n);

  private:
    /* Backing storage (externally provided — typically SDRAM). */
    float*        buf_       = nullptr;
    std::size_t   write_idx_ = 0;

    /* Diffuser state: 4 small all-pass filters in series.  `ap_buf_len_`
     * is the size of the backing SDRAM buffer (the maximum reachable
     * loop length); the *active* loop length is computed per-sample
     * from bloom + LFO and read with linear interpolation.            */
    float*        ap_buf_[kNumAllpasses]     = {nullptr, nullptr, nullptr, nullptr};
    std::size_t   ap_buf_len_[kNumAllpasses] = {0, 0, 0, 0};
    std::size_t   ap_idx_[kNumAllpasses]     = {0, 0, 0, 0};

    /* Per-stage low-frequency oscillator phase (radians, in [0, 2π)).
     * Used by the diffuser to slowly modulate the all-pass loop length
     * when bloom > 0.  `lfo_phase_offset_` is a per-line constant added
     * once at Init so different (line, channel) pairs are out of phase
     * with each other and produce stereo / dual-line motion.          */
    float         lfo_phase_[kNumAllpasses] = {0.0f, 0.0f, 0.0f, 0.0f};
    float         lfo_phase_offset_         = 0.0f;
    /* Latch updated once per block: do we run the modulated long-AP
     * path, or the cheap fixed-length path?  Mirrors the per-V1
     * "shimmer-active" gate so that bloom ≈ 0 costs the same as the
     * old fixed diffuser.                                              */
    bool          bloom_active_ = false;

    /* Damping low-pass (1-pole) state. */
    float         damp_z1_ = 0.0f;

    /* Smoothed parameters (audio-rate one-pole). */
    float         delay_samples_       = 4800.0f;       /* current */
    float         delay_samples_target_= 4800.0f;       /* knob   */
    float         feedback_            = 0.0f;
    float         feedback_target_     = 0.0f;
    float         mix_                 = 0.0f;
    float         mix_target_          = 0.0f;
    float         diffusion_           = 0.0f;
    float         diffusion_target_    = 0.0f;
    float         damping_             = 0.0f;
    float         damping_target_      = 0.0f;
    float         bloom_               = 0.0f;
    float         bloom_target_        = 0.0f;

    /* Read one sample with linear interpolation `delay_samples` taps
     *  back from `write_idx`.  `write_idx` is passed explicitly because
     *  the audio loop holds a local copy that advances every iteration —
     *  using the stale member value would freeze the read pointer for
     *  the whole block and zero-order-hold the wet output at fs/blocksize. */
    float ReadInterp(std::size_t write_idx, float delay_samples) const;

    /* Run one sample through the 4-stage diffuser using the supplied
     *  all-pass coefficient `g` and bloom amount.  When `g == 0` the
     *  all-pass collapses to identity.  When `bloom == 0` and
     *  `bloom_active == false` the loop length is the static
     *  `kApMin*` value (matching the V1 fixed diffuser exactly); when
     *  `bloom > 0` the active length is interpolated toward `kApMax*`
     *  with a slow per-stage LFO on top.                              */
    float Diffuse(float x, float g, float bloom, bool bloom_active);
};

/* ── Two-line, series-stereo engine ─────────────────────────────── */

class DualDelay
{
  public:
    void Init();

    /** Per-line targets (called from the UI loop, slewed in audio). */
    void SetLineParams(uint8_t line_idx,
                       float   delay_seconds,
                       float   feedback,
                       float   mix,
                       float   diffusion,
                       float   damping,
                       float   bloom);

    /** Stereo block.  `in_l`/`in_r` and `out_l`/`out_r` may alias. */
    void ProcessBlock(const float* in_l, const float* in_r,
                      float*       out_l, float*       out_r,
                      std::size_t  n);

    /** Read-only access for UI metering / debug. */
    float Lvl() const { return last_peak_; }

  private:
    /* Two lines × two channels = 4 delay engines (series stereo,
     * per-channel topology: L → D1L → D2L → outL).                  */
    DelayLine line1_l_;
    DelayLine line1_r_;
    DelayLine line2_l_;
    DelayLine line2_r_;

    /* Track peak for VU / future use. */
    float     last_peak_ = 0.0f;
};

} // namespace AthanorDsp

#endif /* DSP_H */
