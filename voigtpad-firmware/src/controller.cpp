/**
 * @file controller.cpp
 * @brief Voigtpad UI controller — see controller.h.
 */

#include "controller.h"

#include <algorithm>
#include <cmath>

namespace VoigtpadUi {

/* ── Mappers ─────────────────────────────────────────────────────── */

float Controller::RootMidiFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return 24.0f + v * 36.0f;        /* 24..60 */
}

float Controller::DetuneCentsFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v * 30.0f;                /* 0..30 */
}

uint8_t Controller::ChordFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    int idx = static_cast<int>(v * 5.0f);
    if (idx > 4) idx = 4;
    return static_cast<uint8_t>(idx);
}

float Controller::ShimmerDriftFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    /* Linear, matches Faust patch range 0.01 .. 1.0 Hz. */
    return 0.01f + v * 0.99f;
}

uint8_t Controller::ShimmerOctaveFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    int idx = static_cast<int>(v * 5.0f);
    if (idx > 4) idx = 4;
    return static_cast<uint8_t>(idx);
}

float Controller::FogCutoffFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return 200.0f + v * 7800.0f;     /* 200..8000 Hz */
}

/* Inverse mappers used when seeding defaults so a user-stated default
 * (e.g. "detune = 15 cents") is round-tripped exactly through the pot
 * normalisation.                                                      */
static float NormFromRange(float v, float lo, float hi)
{
    if (hi <= lo) return 0.0f;
    float n = (v - lo) / (hi - lo);
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

static float NormFromDiscrete(uint8_t idx, uint8_t num_opts)
{
    /* Place the stored norm at the *centre* of the band so the
     * discrete-band band-detector lands exactly on `idx`.            */
    if (num_opts == 0) return 0.0f;
    return (static_cast<float>(idx) + 0.5f) / static_cast<float>(num_opts);
}

/* ── Lifecycle ──────────────────────────────────────────────────── */

void Controller::Init(VoigtpadDsp::GasChords* engine)
{
    engine_                   = engine;
    page_                     = Page::One;
    defer_active_             = false;

    /* Page 1 defaults. */
    pots_[0][0].stored = NormFromRange(36.0f, 24.0f, 60.0f);   /* root MIDI 36 */
    pots_[0][1].stored = 0.50f;                                /* shimmer level */
    pots_[0][2].stored = NormFromRange(15.0f, 0.0f, 30.0f);    /* detune 15c */
    pots_[0][3].stored = 0.50f;                                /* main level */
    pots_[0][4].stored = NormFromDiscrete(0, 5);               /* chord = min */
    pots_[0][5].stored = 0.50f;                                /* sub level */

    /* Page 2 defaults. */
    pots_[1][0].stored = NormFromRange(0.15f, 0.01f, 1.0f);    /* shmr drift 0.15Hz */
    pots_[1][1].stored = NormFromDiscrete(2, 5);               /* shmr oct = 2 */
    pots_[1][2].stored = 0.50f;                                /* shmr air */
    pots_[1][3].stored = NormFromRange(4450.0f, 200.0f, 8000.0f); /* fog 4450 Hz */
    pots_[1][4].stored = 0.20f;                                /* sub warmth */
    pots_[1][5].stored = 0.50f;                                /* unused */

    for (uint8_t pg = 0; pg < kNumPages; pg++)
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            pots_[pg][p].caught       = true;
            pots_[pg][p].sign_at_lock = 0;
        }

    PushParamsToEngine();
}

/* ── Pot-catch ──────────────────────────────────────────────────── */

void Controller::UpdateCatch(uint8_t pot_idx, float phys)
{
    PotState& s = pots_[static_cast<uint8_t>(page_)][pot_idx];

    if (s.caught)
    {
        s.stored = phys;
        return;
    }

    const float diff = phys - s.stored;
    if (std::fabs(diff) <= kCatchTolerance)
    {
        s.caught = true;
        s.stored = phys;
        return;
    }

    const int cur_sign = (diff > 0.0f) ? 1 : -1;
    if (s.sign_at_lock != 0 && cur_sign != s.sign_at_lock)
    {
        s.caught = true;
        s.stored = phys;
        return;
    }
}

/* ── Update ─────────────────────────────────────────────────────── */

void Controller::Update(const float pot_values[NUM_POTS],
                        bool        b1_rising_edge,
                        bool        b2_held)
{
    for (uint8_t p = 0; p < NUM_POTS; p++)
        last_phys_[p] = pot_values[p];

    /* Page advance — single press cycles 1 ↔ 2. */
    if (b1_rising_edge)
    {
        page_ = (page_ == Page::One) ? Page::Two : Page::One;

        /* Re-arm pot-catch on the new page. */
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            PotState&   s    = pots_[static_cast<uint8_t>(page_)][p];
            const float phys = pot_values[p];
            const float diff = phys - s.stored;
            if (std::fabs(diff) <= kCatchTolerance)
            {
                s.caught       = true;
                s.sign_at_lock = 0;
                s.stored       = phys;
            }
            else
            {
                s.caught       = false;
                s.sign_at_lock = (diff > 0.0f) ? 1 : -1;
            }
        }
    }

    /* Deferred-apply state machine driven by B2: while held, suppress
     * engine pushes; on release the very next push commits the
     * deferred values.  No "edge" flag is needed — the engine slews
     * internally, so calling PushParamsToEngine every frame outside
     * the held window is indistinguishable from a single push at the
     * moment of release.                                              */
    defer_active_ = b2_held;

    /* Per-pot catch + tracking on the active page (always — the
     * `stored` value is the visual representation of "where the
     * parameter is going to land", whether B2 is held or not).        */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        UpdateCatch(p, pot_values[p]);

    /* Push live values into the audio engine *unless* B2 is currently
     * held.  Once released, the next call commits the now-stored pot
     * positions; the engine's per-parameter slewing absorbs the jump.
     */
    if (!defer_active_)
        PushParamsToEngine();
}

void Controller::PushParamsToEngine()
{
    if (!engine_) return;

    const PotState (&pg1)[NUM_POTS] = pots_[0];
    const PotState (&pg2)[NUM_POTS] = pots_[1];

    /* Page 1. */
    engine_->SetRootMidi    (RootMidiFromNorm   (pg1[0].stored));
    engine_->SetShimmerLevel(                    pg1[1].stored );
    engine_->SetDetuneCents (DetuneCentsFromNorm(pg1[2].stored));
    engine_->SetMainLevel   (                    pg1[3].stored );
    engine_->SetChordIndex  (ChordFromNorm      (pg1[4].stored));
    engine_->SetSubLevel    (                    pg1[5].stored );

    /* Page 2. */
    engine_->SetShimmerDrift (ShimmerDriftFromNorm  (pg2[0].stored));
    engine_->SetShimmerOctave(ShimmerOctaveFromNorm (pg2[1].stored));
    engine_->SetShimmerAir   (                       pg2[2].stored );
    engine_->SetFogCutoff    (FogCutoffFromNorm     (pg2[3].stored));
    engine_->SetSubWarmth    (                       pg2[4].stored );
    /* pg2[5] reserved. */
}

/* ── Render ─────────────────────────────────────────────────────── */
/*
 * The panel's job is to answer "what is the engine doing right now?"
 * at a glance — so each pot has a renderer tailored to the parameter
 * it controls instead of a uniform arc-fill.  Catalogue (page 1 / 2):
 *
 *   Page 1 (purple-ish identity)
 *     P1  Root pitch     → arc fill (cool violet)
 *     P2  Shimmer level  → V/U meter driven by ShimmerPeak()
 *     P3  Detune         → arc fill (warm orange)
 *     P4  Main level     → V/U meter driven by MainPeak()
 *     P5  Chord          → discrete bands, each chord type has its
 *                          own colour identity; selected band lit.
 *     P6  Sub level      → V/U meter driven by SubPeak()
 *
 *   Page 2 (cyan-ish identity)
 *     P1  Shimmer drift  → dim background arc + a single bright pip
 *                          chasing the engine's shimmer-drift LFO
 *                          phase, so you can read both the rate and
 *                          the current LFO position at a glance.
 *     P2  Shimmer octave → "thermometer" — zones lit from bottom up
 *                          to and including the selected one, hue
 *                          ramping red → blue with rising octave.
 *     P3  Shimmer air    → sparse white sparkles, spawn rate scales
 *                          with the parameter (low ≈ a few per second,
 *                          high ≈ continuous shimmer).
 *     P4  Fog cutoff     → arc fill (cyan).
 *     P5  Sub warmth     → arc fill, hue mixes white → red as the
 *                          parameter rises.
 *     P6  (reserved)     → faint cyan arc.
 *
 * Common UX:
 *   • White pip on the stored value when the pot is uncaught.
 *   • Whole-arc dim + breathe while B2 is held (deferred apply).
 */

namespace {

constexpr uint8_t kArcLeds    = 13;
constexpr float   kArcStartH  = 7.5f;
constexpr float   kArcStepH   = 0.75f;

inline float ValueToHour(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    constexpr float kSpan = static_cast<float>(kArcLeds - 1) * kArcStepH;
    return kArcStartH + v * kSpan;
}

inline uint8_t ValueToFillSteps(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    const uint8_t lit = static_cast<uint8_t>(v * static_cast<float>(kArcLeds) + 0.5f);
    return lit;
}

inline LedPanel::Rgb ScaleGlobal(const LedPanel::Rgb& c)
{
    return LedPanel::Scale(c, kGlobalBrightness);
}

/* Returns the discrete option count for the (page, pot) pair, or 0
 * if it is a continuous parameter.                                    */
inline uint8_t DiscreteOptionsFor(Page pg, uint8_t pot_idx)
{
    if (pg == Page::One && kPage1Discrete.pot_idx == static_cast<int8_t>(pot_idx))
        return kPage1Discrete.num_options;
    if (pg == Page::Two && kPage2Discrete.pot_idx == static_cast<int8_t>(pot_idx))
        return kPage2Discrete.num_options;
    return 0;
}

/* ── Per-chord band colour (P1P5) ────────────────────────────────
 * Mood-coded: minor cool/blue, major warm/gold, min7 violet/jazzy,
 * sus2 cyan/open, sus4 amber/suspended.                            */
constexpr LedPanel::Rgb kChordColors[5] = {
    {0x30, 0x50, 0xFF},   /* min  — cobalt blue   */
    {0xFF, 0xC8, 0x10},   /* maj  — gold          */
    {0xA0, 0x40, 0xFF},   /* min7 — violet        */
    {0x10, 0xE0, 0xC0},   /* sus2 — turquoise     */
    {0xFF, 0x80, 0x20},   /* sus4 — amber         */
};

/* ── Per-octave thermometer colour (P2P2) ────────────────────────
 * Red (warm/low) → blue (cool/high) ramp, one swatch per octave.   */
constexpr LedPanel::Rgb kOctaveColors[5] = {
    {0xFF, 0x40, 0x20},   /* 0 — red       */
    {0xFF, 0xA0, 0x10},   /* 1 — orange    */
    {0xF0, 0xF0, 0x20},   /* 2 — yellow    */
    {0x40, 0xE0, 0x60},   /* 3 — green     */
    {0x40, 0x80, 0xFF},   /* 4 — blue      */
};

/* Map an arc step (0..kArcLeds-1) to a discrete band index. */
inline uint8_t StepToBand(uint8_t step, uint8_t num_opts)
{
    if (num_opts == 0) return 0;
    uint8_t b = static_cast<uint8_t>(
        (static_cast<uint16_t>(step) * num_opts) / kArcLeds);
    if (b >= num_opts) b = num_opts - 1;
    return b;
}

/* ── V/U meter colour ramp.
 * Green → yellow → red across [0, 1.2].  We extend past 1.0 so a
 * brief overshoot above the soft-clip threshold actually shows up
 * as the topmost LED going red, not just "the bar is full".       */
inline LedPanel::Rgb VuColorAt(float fraction)
{
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    /* 0 → green (h=0.33), 0.7 → yellow (h=0.16), 1.0 → red (h=0.0).
     * Linear hue interpolation over a single segment of the wheel. */
    const float h = 0.33f * (1.0f - fraction);
    return LedPanel::Hsv(h, 0.95f, 1.0f);
}

} // namespace

/* ── Per-pot draw helpers ───────────────────────────────────────── */

void Controller::DrawArc(uint8_t pot, float value, LedPanel::Rgb color,
                         float k_pot)
{
    constexpr LedPanel::Rgb kArcOff = {0x06, 0x06, 0x06};
    const uint8_t fill = ValueToFillSteps(value);
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        const LedPanel::Rgb c =
            (step < fill)
                ? LedPanel::Scale(color, k_pot)
                : kArcOff;
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
}

void Controller::DrawVu(uint8_t pot, float level, float k_pot)
{
    /* Map the band's audible level (0..~1.2 with headroom) onto the
     * 13-LED arc.  Below-threshold LEDs sit at a low "off" colour so
     * the arc shape itself communicates "this is a meter".            */
    constexpr LedPanel::Rgb kArcOff = {0x04, 0x06, 0x04};
    const float    norm = level / 1.0f;          /* 1.0 ≈ full clip   */
    const uint8_t  fill = ValueToFillSteps(norm);
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour    = kArcStartH + step * kArcStepH;
        const uint8_t off     = LedPanel::RingOffsetForHour(pot, hour);
        const float   step_fr = static_cast<float>(step + 1)
                              / static_cast<float>(kArcLeds);
        const LedPanel::Rgb c =
            (step < fill)
                ? LedPanel::Scale(VuColorAt(step_fr), k_pot)
                : kArcOff;
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
}

void Controller::DrawChord(uint8_t pot, uint8_t selected, float k_pot)
{
    constexpr uint8_t kN = 5;
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        const uint8_t band = StepToBand(step, kN);

        /* Chord-character colour for this band.  Selected band gets
         * full colour; non-selected bands sit at ~10% so the user can
         * still see the "menu" but the active option is unambiguous. */
        const float band_k = (band == selected) ? 1.0f : 0.10f;
        const LedPanel::Rgb c =
            LedPanel::Scale(kChordColors[band], band_k * k_pot);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
}

void Controller::DrawOctave(uint8_t pot, uint8_t selected, float k_pot)
{
    /* Vertical thermometer: the selected band is at full intensity,
     * every band *below* it is at half intensity, every band *above*
     * is dim — so the silhouette grows upward with octave choice and
     * the colour at the top of the lit zone tells you the value. */
    constexpr uint8_t kN = 5;
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        const uint8_t band = StepToBand(step, kN);

        float band_k;
        if (band == selected)      band_k = 1.0f;
        else if (band < selected)  band_k = 0.40f;
        else                       band_k = 0.06f;

        const LedPanel::Rgb c =
            LedPanel::Scale(kOctaveColors[band], band_k * k_pot);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
}

void Controller::DrawDriftPip(uint8_t pot, float drift_phase, float k_pot)
{
    /* Background: very dim full ring so the pot ring is recognisable
     * even when the pip is on the far side.  Foreground: a single
     * bright pixel chasing the LFO phase, with the two neighbours
     * dimmed for a small "comet tail" — easier to track at slow
     * drift rates than a single dot. */
    constexpr LedPanel::Rgb kBg   = {0x02, 0x05, 0x02};
    constexpr LedPanel::Rgb kPip  = {0x40, 0xFF, 0x80};

    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(kBg));
    }

    /* Map [0,1) phase onto the 13-LED arc and draw a two-tap
     * interpolated pip — at sub-LED phases the brightest light sits
     * between two adjacent LEDs, so the motion looks smooth even at
     * fast drift rates.                                              */
    if (drift_phase < 0.0f) drift_phase = 0.0f;
    if (drift_phase >= 1.0f) drift_phase -= std::floor(drift_phase);

    const float   center_f = drift_phase * static_cast<float>(kArcLeds);
    const int     center_i = static_cast<int>(center_f);
    const float   frac     = center_f - static_cast<float>(center_i);

    auto setStep = [&](int step, float k) {
        if (step < 0 || step >= kArcLeds) return;
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        const LedPanel::Rgb c = LedPanel::Scale(kPip, k * k_pot);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    };

    const int s0 = center_i;
    const int s1 = center_i + 1;
    setStep(s0, 1.0f - frac);
    setStep(s1, frac);
    /* Soft tails one step further out — eases the motion. */
    if (s0 - 1 >= 0)         setStep(s0 - 1, 0.20f * (1.0f - frac));
    if (s1 + 1 <  kArcLeds)  setStep(s1 + 1, 0.20f * frac);
}

void Controller::DrawSparkle(uint8_t pot, float air_param,
                             uint32_t dt_ms, float k_pot)
{
    /* Background: nearly black so individual sparkles read.        */
    constexpr LedPanel::Rgb kBg = {0x02, 0x02, 0x04};
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(kBg));
    }

    /* Subtle by design: 0 sparkles per second at param=0,
     * up to ~12 per second at param=1.  Probabilistic spawn each
     * frame; spawn rate = lambda * dt.                              */
    if (air_param < 0.0f) air_param = 0.0f;
    if (air_param > 1.0f) air_param = 1.0f;
    const float lambda_per_s = air_param * 12.0f;
    const float p_spawn      = lambda_per_s
                             * (static_cast<float>(dt_ms) * 0.001f);

    /* xorshift32 for the random LED choice. */
    auto rand_u32 = [&]() {
        uint32_t x = sparkle_rng_;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        sparkle_rng_ = x;
        return x;
    };
    auto rand_unit = [&]() {
        return static_cast<float>(rand_u32()) * (1.0f / 4294967296.0f);
    };

    /* Decay any active sparkles. */
    for (uint8_t s = 0; s < kArcLeds; s++)
    {
        if (sparkle_intensity_[s] > 0.0f)
        {
            /* Fade ~80% per 100 ms so each sparkle lasts ~250 ms. */
            const float decay = std::pow(0.20f,
                static_cast<float>(dt_ms) * 0.01f);
            sparkle_intensity_[s] *= decay;
            if (sparkle_intensity_[s] < 0.01f)
                sparkle_intensity_[s] = 0.0f;
        }
    }

    /* Spawn 0..N new sparkles based on Poisson-ish process: just
     * roll until we run out of probability mass.  Capped so a
     * pathological dt_ms can't burn the whole loop budget.          */
    float p_remaining = p_spawn;
    for (uint8_t guard = 0; guard < 4 && p_remaining > 0.0f; guard++)
    {
        if (rand_unit() >= p_remaining) break;
        const uint8_t s = static_cast<uint8_t>(rand_u32() % kArcLeds);
        /* Random brightness 0.5..1.0 — a little variation helps the
         * effect feel organic.                                       */
        const float intensity = 0.5f + 0.5f * rand_unit();
        if (intensity > sparkle_intensity_[s])
            sparkle_intensity_[s] = intensity;
        p_remaining -= 1.0f;
    }

    /* Paint sparkles on top of the background.                      */
    constexpr LedPanel::Rgb kSparkle = {0xFF, 0xFF, 0xFF};
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float k = sparkle_intensity_[step];
        if (k <= 0.0f) continue;
        const float   hour = kArcStartH + step * kArcStepH;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        const LedPanel::Rgb c = LedPanel::Scale(kSparkle, k * k_pot);
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
}

void Controller::DrawWarmthArc(uint8_t pot, float value, float k_pot)
{
    /* Arc fill, but the colour mixes from a cool off-white at value=0
     * to a deep red at value=1.  This makes "sub warmth" legible at
     * a glance without reading the fill amount itself.              */
    constexpr LedPanel::Rgb kCool = {0xC0, 0xC0, 0xE0};
    constexpr LedPanel::Rgb kHot  = {0xFF, 0x20, 0x10};
    const LedPanel::Rgb     col   = LedPanel::Mix(kCool, kHot, value);
    DrawArc(pot, value, col, k_pot);
}

void Controller::Render(uint32_t t_ms,
                        const float pot_values[NUM_POTS])
{
    (void)pot_values;

    constexpr LedPanel::Rgb kPagePurple = {0x80, 0x30, 0xFF};
    constexpr LedPanel::Rgb kPageCyan   = {0x10, 0xE0, 0xFF};
    constexpr LedPanel::Rgb kPipWhite   = {0xFF, 0xFF, 0xFF};

    /* Per-pot identity colours used by the Arc / DrawArc-driven
     * pots.  These intentionally diverge from the page colour so
     * each pot has its own visual handle even on the same page. */
    constexpr LedPanel::Rgb kP1RootCol   = {0x80, 0x40, 0xFF};   /* violet  */
    constexpr LedPanel::Rgb kP1DetuneCol = {0xFF, 0x90, 0x30};   /* amber   */
    constexpr LedPanel::Rgb kP2FogCol    = {0x20, 0xC8, 0xFF};   /* sky     */

    const bool                 pg_one  = (page_ == Page::One);
    const LedPanel::Rgb        pgcol   = pg_one ? kPagePurple : kPageCyan;

    /* Frame timing.  Render() runs at ~60 Hz from the main loop;
     * compute the delta for animations that need it (sparkle).     */
    const uint32_t dt_ms = (last_render_ms_ == 0)
        ? 16
        : (t_ms - last_render_ms_);
    last_render_ms_ = t_ms;

    /* Breathing modulator for the deferred-apply indicator.        */
    constexpr float kBreatheHz = 1.0f / 1.5f;
    const float     breathe    = 0.65f
        + 0.35f * std::sin(6.2831853f * kBreatheHz
                           * static_cast<float>(t_ms) * 0.001f);

    LedPanel::Clear();

    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        const PotState& s        = pots_[static_cast<uint8_t>(page_)][p];
        const uint8_t   num_opts = DiscreteOptionsFor(page_, p);

        /* Per-pot brightness factor:
         *   • Caught     → 1.0
         *   • Uncaught   → 0.30 (so the white pip is unambiguous)
         *   • B2 held    → multiplied by `breathe` (sin LFO)         */
        float k_pot = s.caught ? 1.0f : 0.30f;
        if (defer_active_) k_pot *= breathe;

        /* Dispatch: per-(page, pot) animation. */
        if (pg_one)
        {
            switch (p)
            {
                case 0: DrawArc(p, s.stored, kP1RootCol, k_pot);           break;
                case 1: DrawVu(p, engine_ ? engine_->ShimmerPeak() : 0.0f,
                               k_pot);                                     break;
                case 2: DrawArc(p, s.stored, kP1DetuneCol, k_pot);         break;
                case 3: DrawVu(p, engine_ ? engine_->MainPeak() : 0.0f,
                               k_pot);                                     break;
                case 4: DrawChord(p,
                            static_cast<uint8_t>(std::min<int>(
                                kPage1Discrete.num_options - 1,
                                static_cast<int>(s.stored
                                    * kPage1Discrete.num_options))),
                            k_pot);                                        break;
                case 5: DrawVu(p, engine_ ? engine_->SubPeak() : 0.0f,
                               k_pot);                                     break;
                default: break;
            }
        }
        else
        {
            switch (p)
            {
                case 0: DrawDriftPip(p,
                            engine_ ? engine_->DriftPhase() : 0.0f,
                            k_pot);                                        break;
                case 1: DrawOctave(p,
                            static_cast<uint8_t>(std::min<int>(
                                kPage2Discrete.num_options - 1,
                                static_cast<int>(s.stored
                                    * kPage2Discrete.num_options))),
                            k_pot);                                        break;
                case 2: DrawSparkle(p, s.stored, dt_ms, k_pot);            break;
                case 3: DrawArc(p, s.stored, kP2FogCol, k_pot);            break;
                case 4: DrawWarmthArc(p, s.stored, k_pot);                 break;
                default: DrawArc(p, s.stored, pgcol, k_pot * 0.4f);        break;
            }
        }

        /* White pip for uncaught pots, drawn last so it wins over the
         * underlying animation.  Suppressed for discrete params (the
         * highlighted band already conveys the stored value).        */
        if (!s.caught && num_opts == 0)
        {
            const float   pip_hour = ValueToHour(s.stored);
            const uint8_t pip_off  = LedPanel::RingOffsetForHour(p, pip_hour);
            LedPanel::Rgb pip = kPipWhite;
            if (defer_active_) pip = LedPanel::Scale(pip, breathe);
            LedPanel::SetRingByOffset(p, pip_off, ScaleGlobal(pip));
        }
    }

    /* Button accent LEDs:
     *   B1 → page colour, dim/lit so the page is visible at a glance.
     *   B2 → breathes brightly while held, dim resting state.
     *   B3 → reserved (very dim).                                    */
    LedPanel::SetButtonPair(0, ScaleGlobal(LedPanel::Scale(pgcol, 0.55f)));

    {
        constexpr LedPanel::Rgb kB2Hold = {0xFF, 0xC0, 0x40}; /* warm amber */
        const LedPanel::Rgb     b2col   = defer_active_
            ? LedPanel::Scale(kB2Hold, breathe)
            : LedPanel::Rgb{0x10, 0x08, 0x02};
        LedPanel::SetButtonPair(1, ScaleGlobal(b2col));
    }

    constexpr LedPanel::Rgb kDim = {0x05, 0x05, 0x05};
    LedPanel::SetButtonPair(2, ScaleGlobal(kDim));
}

} // namespace VoigtpadUi
