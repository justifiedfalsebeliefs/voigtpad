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
 * Layout/animation parallels athanor's Render():
 *   • The ring arc spans 7:30 → 4:30 in 13 LEDs (0.75 h pitch).
 *   • Page 1 colour = purple (#8030FF), Page 2 = cyan (#10E0FF).
 *   • Discrete-option pots draw 5 hue bands instead of a filled arc,
 *     with the selected band brightened.
 *   • While B2 is held, every active arc breathes (sin LFO).
 *   • White pip on the stored value when the pot is uncaught.
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

/* Hue for discrete option `idx` of `num`, evenly distributed across
 * the colour wheel.  Skip the last 1/num so the first/last hues
 * remain distinguishable.                                             */
inline LedPanel::Rgb DiscreteBandColor(uint8_t idx, uint8_t num)
{
    const float h = (num > 0) ? (static_cast<float>(idx) / static_cast<float>(num)) : 0.0f;
    return LedPanel::Hsv(h, 0.85f, 1.0f);
}

inline uint8_t StepToBand(uint8_t step, uint8_t num_opts)
{
    if (num_opts == 0) return 0;
    /* Map the 13-LED arc onto N bands.  Use floor(step * N / 13).    */
    uint8_t b = static_cast<uint8_t>((static_cast<uint16_t>(step) * num_opts) / kArcLeds);
    if (b >= num_opts) b = num_opts - 1;
    return b;
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

} // namespace

void Controller::Render(uint32_t t_ms,
                        const float pot_values[NUM_POTS])
{
    (void)pot_values;

    constexpr LedPanel::Rgb kPagePurple = {0x80, 0x30, 0xFF};
    constexpr LedPanel::Rgb kPageCyan   = {0x10, 0xE0, 0xFF};
    constexpr LedPanel::Rgb kPipWhite   = {0xFF, 0xFF, 0xFF};
    constexpr LedPanel::Rgb kArcDim     = {0x06, 0x06, 0x06};

    const bool                 pg_one  = (page_ == Page::One);
    const LedPanel::Rgb        pgcol   = pg_one ? kPagePurple : kPageCyan;

    /* Breathing modulator — slow sine on t_ms for the deferred-apply
     * indicator.  Period ≈ 1.5 s, depth 35 %.                        */
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

        if (num_opts > 0)
        {
            /* ── Discrete-band rendering. ─────────────────────────── */
            const uint8_t selected = static_cast<uint8_t>(
                std::min<int>(num_opts - 1,
                              static_cast<int>(s.stored * num_opts)));

            for (uint8_t step = 0; step < kArcLeds; step++)
            {
                const float   hour = kArcStartH + step * kArcStepH;
                const uint8_t off  = LedPanel::RingOffsetForHour(p, hour);
                const uint8_t band = StepToBand(step, num_opts);

                LedPanel::Rgb base = DiscreteBandColor(band, num_opts);
                /* Non-selected bands sit dim; selected band is
                 * "slightly brighter" (per spec), not blown out, so
                 * the user can scan colour zones without losing track
                 * of the others.                                      */
                const float band_k = (band == selected) ? 0.95f : 0.30f;
                const LedPanel::Rgb c = LedPanel::Scale(base, band_k * k_pot);
                LedPanel::SetRingByOffset(p, off, ScaleGlobal(c));
            }
        }
        else
        {
            /* ── Continuous arc rendering. ───────────────────────── */
            const uint8_t fill = ValueToFillSteps(s.stored);
            for (uint8_t step = 0; step < kArcLeds; step++)
            {
                const float   hour = kArcStartH + step * kArcStepH;
                const uint8_t off  = LedPanel::RingOffsetForHour(p, hour);
                const LedPanel::Rgb c =
                    (step < fill)
                        ? LedPanel::Scale(pgcol, k_pot)
                        : kArcDim;
                LedPanel::SetRingByOffset(p, off, ScaleGlobal(c));
            }
        }

        /* White pip for uncaught pots, drawn last to win against the
         * underlying arc.  Suppressed for discrete pots (the
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
