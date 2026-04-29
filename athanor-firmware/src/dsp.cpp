/**
 * @file dsp.cpp
 * @brief V1 dual-delay implementation — multi-tap delay + Schroeder
 *        diffuser + damped feedback.
 *
 * Crash- and overflow-safety:
 *   • Delay buffer length is a power of two, addressed with a bitmask
 *     so the write / read indices can never run off the end.
 *   • The delay-time control is clamped to
 *     [kMinDelaySeconds, kMaxDelaySeconds] before being converted to a
 *     sample count, and is then re-clamped to
 *     [1.0, kDelayBufSize - 2] so the linear-interp read always lands
 *     inside the buffer (one-sample headroom each side).
 *   • Feedback is clamped to [0, 1.10] (per spec, "controlled self-
 *     oscillation territory"); the feedback path is also damped by a
 *     one-pole low-pass and a tanh-style soft-clip so the DC and HF
 *     gain at unity is well below 1, preventing run-away.
 *
 * Click/zipper-noise mitigation:
 *   • Every control (delay samples, feedback, mix) is target-tracked
 *     with a per-sample one-pole.  ~30 ms time constant.
 *   • The delay tap is read with linear interpolation, so smoothly
 *     changing delay time produces a smooth pitch-glide rather than
 *     stepping artefacts.
 */

#include "dsp.h"

#include "daisy_seed.h"
#include "hardware.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace AthanorDsp {

/* ─────────────────────────────────────────────────────────────────── */
/*  SDRAM-resident buffers                                             */
/* ─────────────────────────────────────────────────────────────────── */
/*
 * Four delay buffers (Line 1 L, Line 1 R, Line 2 L, Line 2 R) live in
 * external SDRAM.  Daisy provides 64 MB of SDRAM via the QSPI/FMC bus,
 * so 4 × 131 072 floats = 2 MB is comfortably within budget.
 *
 * The 8 small diffuser buffers are SDRAM too — they are accessed
 * sample-rate but the indices are sequential, so the cache prefetcher
 * handles them efficiently.
 */
static float DSY_SDRAM_BSS s_buf_l1l[kDelayBufSize];
static float DSY_SDRAM_BSS s_buf_l1r[kDelayBufSize];
static float DSY_SDRAM_BSS s_buf_l2l[kDelayBufSize];
static float DSY_SDRAM_BSS s_buf_l2r[kDelayBufSize];

/* Schroeder all-pass loop sizes (samples).  See `dsp.h` for the
 * design rationale.  At bloom = 0 the active loop length is the
 * `kApMin*` value (short, hand-picked mutually-prime values in the
 * 5–15 ms range — V1 fixed-diffuser behaviour); at bloom = 1 it
 * extends up to the `kApMax*` value (40–120 ms, plate / hall sizes),
 * with a slow per-stage LFO modulating the length on top.  The
 * SDRAM buffers are sized to the maxima so the active length can
 * sweep continuously without reallocation.                            */
static constexpr std::size_t kAp0Min = kApMin0;
static constexpr std::size_t kAp1Min = kApMin1;
static constexpr std::size_t kAp2Min = kApMin2;
static constexpr std::size_t kAp3Min = kApMin3;
static constexpr std::size_t kAp0Max = kApMax0;
static constexpr std::size_t kAp1Max = kApMax1;
static constexpr std::size_t kAp2Max = kApMax2;
static constexpr std::size_t kAp3Max = kApMax3;

/* 4 channels × 4 stages = 16 buffers, each sized to its stage maximum. */
static float DSY_SDRAM_BSS s_ap_l1l_0[kAp0Max];
static float DSY_SDRAM_BSS s_ap_l1l_1[kAp1Max];
static float DSY_SDRAM_BSS s_ap_l1l_2[kAp2Max];
static float DSY_SDRAM_BSS s_ap_l1l_3[kAp3Max];

static float DSY_SDRAM_BSS s_ap_l1r_0[kAp0Max];
static float DSY_SDRAM_BSS s_ap_l1r_1[kAp1Max];
static float DSY_SDRAM_BSS s_ap_l1r_2[kAp2Max];
static float DSY_SDRAM_BSS s_ap_l1r_3[kAp3Max];

static float DSY_SDRAM_BSS s_ap_l2l_0[kAp0Max];
static float DSY_SDRAM_BSS s_ap_l2l_1[kAp1Max];
static float DSY_SDRAM_BSS s_ap_l2l_2[kAp2Max];
static float DSY_SDRAM_BSS s_ap_l2l_3[kAp3Max];

static float DSY_SDRAM_BSS s_ap_l2r_0[kAp0Max];
static float DSY_SDRAM_BSS s_ap_l2r_1[kAp1Max];
static float DSY_SDRAM_BSS s_ap_l2r_2[kAp2Max];
static float DSY_SDRAM_BSS s_ap_l2r_3[kAp3Max];

/* ─────────────────────────────────────────────────────────────────── */
/*  Tunable engine constants                                           */
/* ─────────────────────────────────────────────────────────────────── */

/* All-pass coefficient ceiling for the diffuser.  When the page-2
 * "diffusion" knob is at maximum, each Schroeder all-pass uses this
 * `g`.  0.7 is just shy of the classic 0.707 RT-sqrt-2 value — high
 * enough for a continuous wash but low enough that sustained feedback
 * stays musical without ringing into a metallic tone.                */
static constexpr float kApGainMax = 0.7f;

/* Multi-tap mix.  4 taps, fixed positions as fractions of the delay
 * length, with weights summing to 1.0.  The longest tap (1.0) carries
 * the most energy so the line still sounds like a delay; the shorter
 * taps add the slight ER smear that lets the diffuser do its job.    */
static constexpr float kTapPos[kNumTaps] = {0.21f, 0.43f, 0.71f, 1.00f};
static constexpr float kTapAmp[kNumTaps] = {0.15f, 0.20f, 0.25f, 0.40f};

/* One-pole smoothing coefficients (per-sample).  These set roughly
 * 30 ms time constants at 48 kHz, which is fast enough to track knob
 * gestures and slow enough to mask any zipper noise.                  */
static constexpr float kSmoothFast = 0.0010f;   /* ~21 ms ≈ feedback / mix */
static constexpr float kSmoothTime = 0.0005f;   /* ~42 ms ≈ delay length   */

/* Damping low-pass coefficient in the feedback path.
 * Variable now: damping=0 → α≈1 (passthrough), damping=1 → α≈0.0067
 * (≈ 50 Hz @ 48 kHz, very dark "mossy" tail).
 *   y = y + α·(x - y), with α = exp(-5·damping).                      */
static inline float DampAlpha(float damping)
{
    if (damping <= 0.0f) return 1.0f;
    if (damping >= 1.0f) damping = 1.0f;
    return std::exp(-5.0f * damping);
}

/* Threshold below which the bloom modulation path is bypassed
 * entirely.  Once the smoothed bloom level falls below this, the LFO
 * advance and modulated fractional-read code is skipped and the
 * diffuser falls back to its V1 fixed short-loop behaviour, so the
 * cost at bloom ≈ 0 is identical to the original static diffuser.
 * A small hysteresis prevents thrashing at the threshold.            */
static constexpr float kBloomOnThresh  = 0.005f;
static constexpr float kBloomOffThresh = 0.001f;

/* Per-stage minimum / maximum loop lengths and LFO rates.
 *
 * Frequencies are deliberately mutually-non-harmonic in the 0.1 to
 * 0.3 Hz range — well below any audible pitch modulation.  At the
 * configured ±4 % depth the worst-case sweep across the (max − min)
 * range is < 1 % over a few seconds, i.e. a barely-perceptible
 * "size breathing" rather than a chorus.                              */
static constexpr std::size_t kApMin[kNumAllpasses] = {
    kAp0Min, kAp1Min, kAp2Min, kAp3Min};
static constexpr std::size_t kApMax[kNumAllpasses] = {
    kAp0Max, kAp1Max, kAp2Max, kAp3Max};
static constexpr float       kLfoFreqHz[kNumAllpasses] = {
    0.11f, 0.17f, 0.23f, 0.29f};

/* Modulation depth as a fraction of (max − min), scaled by bloom.    */
static constexpr float kBloomLfoDepthFrac = 0.04f;

/* 2π and per-stage LFO phase increments (radians / sample). */
static constexpr float kTwoPi = 6.28318530717958647692f;
static constexpr float kLfoPhaseInc[kNumAllpasses] = {
    kTwoPi * kLfoFreqHz[0] / kSampleRate,
    kTwoPi * kLfoFreqHz[1] / kSampleRate,
    kTwoPi * kLfoFreqHz[2] / kSampleRate,
    kTwoPi * kLfoFreqHz[3] / kSampleRate,
};

/* Hard saturation in the feedback loop — guarantees the buffer cannot
 * grow without bound even at fb > 1.0 with self-resonance.            */
static inline float SoftClip(float x)
{
    /* tanh-ish, faster than std::tanh on M7 with no FPU lookup. */
    if (x >  1.5f) return  1.0f;
    if (x < -1.5f) return -1.0f;
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  DelayLine                                                          */
/* ─────────────────────────────────────────────────────────────────── */

void DelayLine::Init(float* main_buf,
                     float* ap_buf0, std::size_t ap_buf_len0,
                     float* ap_buf1, std::size_t ap_buf_len1,
                     float* ap_buf2, std::size_t ap_buf_len2,
                     float* ap_buf3, std::size_t ap_buf_len3,
                     float  lfo_phase_offset)
{
    buf_       = main_buf;
    write_idx_ = 0;

    if (buf_)
        std::memset(buf_, 0, kDelayBufSize * sizeof(float));

    ap_buf_[0] = ap_buf0;  ap_buf_len_[0] = ap_buf_len0;
    ap_buf_[1] = ap_buf1;  ap_buf_len_[1] = ap_buf_len1;
    ap_buf_[2] = ap_buf2;  ap_buf_len_[2] = ap_buf_len2;
    ap_buf_[3] = ap_buf3;  ap_buf_len_[3] = ap_buf_len3;

    for (std::size_t i = 0; i < kNumAllpasses; i++)
    {
        if (ap_buf_[i] && ap_buf_len_[i] > 0)
            std::memset(ap_buf_[i], 0, ap_buf_len_[i] * sizeof(float));
        ap_idx_[i] = 0;
    }

    /* Seed each per-stage LFO at a slightly different starting phase
     * — the stage frequencies are already mutually non-harmonic, but
     * staggering the start avoids any brief moment of all-stages-in-
     * phase sweep when the user first turns bloom up.  The per-line
     * `lfo_phase_offset` is added on top so different (line, channel)
     * pairs sit at different points in the slow sweep, giving stereo
     * and dual-line motion.                                           */
    constexpr float kStagePhase[kNumAllpasses] = {
        0.0f,
        kTwoPi * 0.25f,
        kTwoPi * 0.50f,
        kTwoPi * 0.75f,
    };
    for (std::size_t i = 0; i < kNumAllpasses; i++)
    {
        float ph = kStagePhase[i] + lfo_phase_offset;
        while (ph >= kTwoPi) ph -= kTwoPi;
        while (ph <  0.0f)   ph += kTwoPi;
        lfo_phase_[i] = ph;
    }
    lfo_phase_offset_ = lfo_phase_offset;
    bloom_active_     = false;

    damp_z1_ = 0.0f;

    /* Sensible default ≈ 100 ms delay, no feedback, fully dry. */
    delay_samples_        = 0.1f * kSampleRate;
    delay_samples_target_ = delay_samples_;
    feedback_             = 0.0f;
    feedback_target_      = 0.0f;
    mix_                  = 0.0f;
    mix_target_           = 0.0f;
    diffusion_            = 0.0f;
    diffusion_target_     = 0.0f;
    damping_              = 0.0f;
    damping_target_       = 0.0f;
    bloom_                = 0.0f;
    bloom_target_         = 0.0f;
}

void DelayLine::SetParams(float delay_seconds, float feedback, float mix,
                          float diffusion,     float damping,  float bloom)
{
    /* Clamp delay to the physically supported range, then to the
     * buffer-safe range (1 sample of headroom each side for interp). */
    if (delay_seconds < kMinDelaySeconds) delay_seconds = kMinDelaySeconds;
    if (delay_seconds > kMaxDelaySeconds) delay_seconds = kMaxDelaySeconds;

    float ds = delay_seconds * kSampleRate;
    if (ds < 1.0f)
        ds = 1.0f;
    const float ds_max = static_cast<float>(kDelayBufSize - 2);
    if (ds > ds_max)
        ds = ds_max;
    delay_samples_target_ = ds;

    /* Feedback clamp: 0..1.10 per spec.  The damped low-pass + soft-
     * clip on the feedback path keep the loop stable even at 1.10.   */
    if (feedback < 0.0f)  feedback = 0.0f;
    if (feedback > 1.10f) feedback = 1.10f;
    feedback_target_ = feedback;

    /* Mix is a straight 0..1. */
    if (mix < 0.0f) mix = 0.0f;
    if (mix > 1.0f) mix = 1.0f;
    mix_target_ = mix;

    /* Page-2 controls — also straight 0..1. */
    if (diffusion < 0.0f) diffusion = 0.0f;
    if (diffusion > 1.0f) diffusion = 1.0f;
    diffusion_target_ = diffusion;

    if (damping < 0.0f) damping = 0.0f;
    if (damping > 1.0f) damping = 1.0f;
    damping_target_ = damping;

    if (bloom < 0.0f) bloom = 0.0f;
    if (bloom > 1.0f) bloom = 1.0f;
    bloom_target_ = bloom;
}

float DelayLine::ReadInterp(std::size_t write_idx, float delay_samples) const
{
    /* Compute fractional read index relative to the *current* write
     * position (passed in by the caller, since the audio loop walks
     * a local copy of write_idx every iteration).                    */
    const float       idx_f = static_cast<float>(write_idx) - delay_samples;
    const int         idx_i = static_cast<int>(std::floor(idx_f));
    const float       frac  = idx_f - static_cast<float>(idx_i);

    /* Mask handles the negative-wrap case correctly because
     * kDelayBufSize is a power of two and we use unsigned & mask.    */
    const std::size_t i0 = static_cast<std::size_t>(idx_i)     & kDelayBufMask;
    const std::size_t i1 = static_cast<std::size_t>(idx_i + 1) & kDelayBufMask;

    return buf_[i0] + frac * (buf_[i1] - buf_[i0]);
}

float DelayLine::Diffuse(float x, float g, float bloom, bool bloom_active)
{
    /* Series of 4 Schroeder all-pass filters with a runtime-variable
     * coefficient g (page-2 "diffusion" knob) and a runtime-variable
     * loop length per stage (page-2 "bloom" knob).
     *
     *   v       = x + g·delayed
     *   y       = -g·v + delayed
     *   buf[w]  = v
     *
     * `delayed` is the value `len_active` samples back from the write
     * pointer.  When `bloom_active` is false the loop length collapses
     * to the V1 short fixed value `kApMin[s]` and the read is a plain
     * integer index — bit-exact-equivalent to the original V1 path so
     * the cost at bloom = 0 is the same as a static diffuser.  When
     * `bloom_active` is true the length is interpolated up to
     * `kApMax[s]` and a slow per-stage LFO modulates it; the read is
     * done with linear interpolation from a fractional index.        */
    if (!bloom_active)
    {
        for (std::size_t s = 0; s < kNumAllpasses; s++)
        {
            const std::size_t buflen = ap_buf_len_[s];
            if (!ap_buf_[s] || buflen == 0)
                continue;

            const std::size_t len_active = kApMin[s];
            const std::size_t idx        = ap_idx_[s];
            /* Read offset = (idx - len_active) mod buflen.  buflen ≥
             * len_active by construction, so adding buflen - len_active
             * keeps the unsigned arithmetic non-negative.            */
            std::size_t back = idx + buflen - len_active;
            if (back >= buflen) back -= buflen;
            const float delayed = ap_buf_[s][back];

            const float v = x + g * delayed;
            const float y = -g * v + delayed;

            ap_buf_[s][idx] = v;
            std::size_t next = idx + 1u;
            if (next >= buflen) next = 0u;
            ap_idx_[s] = next;

            x = y;
        }
        return x;
    }

    /* bloom_active path. */
    for (std::size_t s = 0; s < kNumAllpasses; s++)
    {
        const std::size_t buflen = ap_buf_len_[s];
        if (!ap_buf_[s] || buflen == 0)
            continue;

        const float lo       = static_cast<float>(kApMin[s]);
        const float hi       = static_cast<float>(kApMax[s]);
        const float range    = hi - lo;
        const float center   = lo + bloom * range;
        const float lfo_amt  = bloom * kBloomLfoDepthFrac * range;

        /* sin from radians; faster than building a table for this rate
         * (≤ 0.3 Hz) and toolchain libm vectorises it.  The LFO update
         * is wrapped to [0, 2π).                                       */
        float ph = lfo_phase_[s];
        const float len_active_f = center + lfo_amt * std::sin(ph);
        ph += kLfoPhaseInc[s];
        if (ph >= kTwoPi) ph -= kTwoPi;
        lfo_phase_[s] = ph;

        /* Clamp to a safe range — the buffer always has 1 sample of
         * headroom each side for the linear interp.                  */
        float la = len_active_f;
        if (la < 1.0f) la = 1.0f;
        const float la_max = static_cast<float>(buflen) - 2.0f;
        if (la > la_max) la = la_max;

        const std::size_t idx = ap_idx_[s];
        const float pos_f = static_cast<float>(idx)
                          + static_cast<float>(buflen)
                          - la;
        /* pos_f is guaranteed >= 0 because buflen − la ≥ 2 ≥ 0; do a
         * single floor + mod via an unsigned cast that's safe.        */
        const std::size_t pos_u = static_cast<std::size_t>(pos_f);
        const float       fr    = pos_f - static_cast<float>(pos_u);
        std::size_t       i0    = pos_u % buflen;
        std::size_t       i1    = i0 + 1u;
        if (i1 >= buflen) i1 = 0u;
        const float delayed = ap_buf_[s][i0]
                            + fr * (ap_buf_[s][i1] - ap_buf_[s][i0]);

        const float v = x + g * delayed;
        const float y = -g * v + delayed;

        ap_buf_[s][idx] = v;
        std::size_t next = idx + 1u;
        if (next >= buflen) next = 0u;
        ap_idx_[s] = next;

        x = y;
    }
    return x;
}

void DelayLine::ProcessBlock(const float* in, float* out, std::size_t n)
{
    /* Cache locals for tighter loop. */
    float       delay_samples       = delay_samples_;
    const float delay_samples_target= delay_samples_target_;
    float       feedback            = feedback_;
    const float feedback_target     = feedback_target_;
    float       mix                 = mix_;
    const float mix_target          = mix_target_;
    float       diffusion           = diffusion_;
    const float diffusion_target    = diffusion_target_;
    float       damping             = damping_;
    const float damping_target      = damping_target_;
    float       bloom               = bloom_;
    const float bloom_target        = bloom_target_;
    float       damp_z1             = damp_z1_;
    std::size_t write_idx           = write_idx_;

    /* Latch the bloom-active flag for the whole block.  The smoothed
     * bloom level decides whether to spend the cycles on the LFO
     * advance + fractional-read code path; when bloom ≈ 0 we fall
     * back to the cheap fixed-length integer-read AP path so the
     * cost is the same as the V1 static diffuser.  A small
     * hysteresis stops on/off thrash from ADC jitter at the
     * threshold.                                                       */
    const float bloom_level = std::fmax(bloom, bloom_target);
    bool bloom_active = bloom_active_;
    if (!bloom_active && bloom_level > kBloomOnThresh)
    {
        bloom_active = true;
    }
    else if (bloom_active && bloom_level < kBloomOffThresh)
    {
        bloom_active = false;
    }

    for (std::size_t i = 0; i < n; i++)
    {
        /* ── 1. Slew control parameters one sample. ── */
        delay_samples += (delay_samples_target - delay_samples) * kSmoothTime;
        feedback      += (feedback_target      - feedback)      * kSmoothFast;
        mix           += (mix_target           - mix)           * kSmoothFast;
        diffusion     += (diffusion_target     - diffusion)     * kSmoothFast;
        damping       += (damping_target       - damping)       * kSmoothFast;
        bloom         += (bloom_target         - bloom)         * kSmoothFast;

        /* ── 2. Delay-buffer read.
         *
         * IMPORTANT: pass the *local* write_idx — the member write_idx_
         * is only synced back at the end of the block, so using it
         * here would freeze the read pointer for the whole block and
         * zero-order-hold the wet output at fs / block_size.          */
        float wet = ReadInterp(write_idx, delay_samples);

        /* ── 3. Diffusion (variable-g, variable-length Schroeder
         * all-pass network).  The "diffusion" knob sets the all-pass
         * coefficient g (0 → identity, kApGainMax → max wash); the
         * "bloom" knob morphs each stage's loop length from V1's
         * short fixed values up to plate / hall sizes, with a slow
         * per-stage LFO modulating the length on top.  At bloom = 0
         * the diffuser is bit-equivalent to V1's static short loops. */
        const float g = diffusion * kApGainMax;
        wet = Diffuse(wet, g, bloom, bloom_active);

        /* ── 4. Damp the feedback signal (one-pole LP, variable α).
         * α = 1 → no damping; α → 0 → very dark mossy tail.          */
        const float damp_alpha = DampAlpha(damping);
        damp_z1   = damp_z1 + damp_alpha * (wet - damp_z1);

        /* ── 5. Feedback path: damped wet, soft-clipped.  No pitch
         * shifter — the "size" and "movement" axis is handled by
         * bloom inside the diffuser, which is musically much more
         * useful than the previous octave-up shimmer.                 */
        const float fb_signal = SoftClip(damp_z1 * feedback);

        /* ── 6. Write input + feedback into the delay buffer. ── */
        const float x = in[i];
        buf_[write_idx & kDelayBufMask] = x + fb_signal;
        write_idx = (write_idx + 1u) & kDelayBufMask;

        /* ── 7. Dry/wet mix to output. ── */
        out[i] = x * (1.0f - mix) + wet * mix;
    }

    /* Persist locals back to members. */
    delay_samples_ = delay_samples;
    feedback_      = feedback;
    mix_           = mix;
    diffusion_     = diffusion;
    damping_       = damping;
    bloom_         = bloom;
    damp_z1_       = damp_z1;
    write_idx_     = write_idx;
    bloom_active_  = bloom_active;
}

/* ─────────────────────────────────────────────────────────────────── */
/*  DualDelay                                                          */
/* ─────────────────────────────────────────────────────────────────── */

void DualDelay::Init()
{
    /* Per-channel LFO phase offsets give natural stereo / dual-line
     * motion when bloom > 0:
     *   line 1 L:   0          line 1 R:   π/2
     *   line 2 L:   π          line 2 R:   3π/2
     * i.e. each channel sits at a different point in the slow sweep,
     * so a stereo signal walks through a moving "size" rather than
     * sitting in a single static room.                                 */
    constexpr float kPi  = 3.14159265358979f;
    constexpr float kPi2 = kPi * 0.5f;

    line1_l_.Init(s_buf_l1l,
                  s_ap_l1l_0, kAp0Max,
                  s_ap_l1l_1, kAp1Max,
                  s_ap_l1l_2, kAp2Max,
                  s_ap_l1l_3, kAp3Max,
                  0.0f);

    line1_r_.Init(s_buf_l1r,
                  s_ap_l1r_0, kAp0Max,
                  s_ap_l1r_1, kAp1Max,
                  s_ap_l1r_2, kAp2Max,
                  s_ap_l1r_3, kAp3Max,
                  kPi2);

    line2_l_.Init(s_buf_l2l,
                  s_ap_l2l_0, kAp0Max,
                  s_ap_l2l_1, kAp1Max,
                  s_ap_l2l_2, kAp2Max,
                  s_ap_l2l_3, kAp3Max,
                  kPi);

    line2_r_.Init(s_buf_l2r,
                  s_ap_l2r_0, kAp0Max,
                  s_ap_l2r_1, kAp1Max,
                  s_ap_l2r_2, kAp2Max,
                  s_ap_l2r_3, kAp3Max,
                  kPi + kPi2);
}

void DualDelay::SetLineParams(uint8_t line_idx,
                              float   delay_seconds,
                              float   feedback,
                              float   mix,
                              float   diffusion,
                              float   damping,
                              float   bloom)
{
    if (line_idx == 0)
    {
        line1_l_.SetParams(delay_seconds, feedback, mix,
                           diffusion, damping, bloom);
        line1_r_.SetParams(delay_seconds, feedback, mix,
                           diffusion, damping, bloom);
    }
    else if (line_idx == 1)
    {
        line2_l_.SetParams(delay_seconds, feedback, mix,
                           diffusion, damping, bloom);
        line2_r_.SetParams(delay_seconds, feedback, mix,
                           diffusion, damping, bloom);
    }
}

void DualDelay::ProcessBlock(const float* in_l, const float* in_r,
                             float*       out_l, float*       out_r,
                             std::size_t  n)
{
    /* Series stereo: in → Delay 1 → Delay 2 → out, per channel.
     *
     * We process one block through each stage in turn, using the
     * output buffer as the intermediate.  Aliasing is fine because
     * we read in[i] before writing out[i] inside DelayLine.          */

    line1_l_.ProcessBlock(in_l, out_l, n);
    line1_r_.ProcessBlock(in_r, out_r, n);

    line2_l_.ProcessBlock(out_l, out_l, n);
    line2_r_.ProcessBlock(out_r, out_r, n);

    /* Track stereo peak for any UI metering. */
    float peak = last_peak_ * 0.92f;
    for (std::size_t i = 0; i < n; i++)
    {
        const float a = std::fabs(out_l[i]);
        const float b = std::fabs(out_r[i]);
        if (a > peak) peak = a;
        if (b > peak) peak = b;
    }
    last_peak_ = peak;
}

} // namespace AthanorDsp
