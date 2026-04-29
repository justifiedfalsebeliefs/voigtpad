/**
 * @file controller.cpp
 * @brief Voigtpad UI controller implementation — see controller.h.
 *
 * Pot-catch logic (UpdateCatch + Update) is preserved verbatim from
 * the athanor controller, per the spec: "Use the existing pot catch
 * implementation."  All other UI behaviours (deferred apply, discrete
 * band rendering, page colours) are new.
 */

#include "controller.h"

#include <algorithm>
#include <cmath>

namespace VoigtpadUi {

/* ─────────────────────────────────────────────────────────────────── */
/*  Pot → engine-unit mappers                                          */
/* ─────────────────────────────────────────────────────────────────── */

static inline float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

float Controller::MidiFromNorm(float v)
{
    v = Clamp01(v);
    /* Linear MIDI 24..60 (3 octaves). */
    return 24.0f + 36.0f * v;
}

float Controller::DetuneCentsFromNorm(float v)
{
    return Clamp01(v) * 30.0f;
}

float Controller::Linear01(float v)
{
    return Clamp01(v);
}

float Controller::DriftHzFromNorm(float v)
{
    /* Log: 0.01..1.0 Hz. */
    v = Clamp01(v);
    constexpr float lo = 0.01f, hi = 1.0f;
    return lo * std::pow(hi / lo, v);
}

float Controller::FogCutoffFromNorm(float v)
{
    /* Log: 200..8000 Hz. */
    v = Clamp01(v);
    constexpr float lo = 200.0f, hi = 8000.0f;
    return lo * std::pow(hi / lo, v);
}

uint8_t Controller::BandIndex(float v, uint8_t n_bands)
{
    v = Clamp01(v);
    /* Discrete bands of equal width across [0, 1]. */
    int b = static_cast<int>(v * static_cast<float>(n_bands));
    if (b < 0)              b = 0;
    if (b >= n_bands)       b = n_bands - 1;
    return static_cast<uint8_t>(b);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Default values (per spec)                                          */
/* ─────────────────────────────────────────────────────────────────── */

namespace {

/* Inverse mappers — used to convert default engineering units back
 * into the 0..1 stored representation.  Only need the inverses we use. */
inline float MidiToNorm(float midi)
{
    /* (midi - 24) / 36 */
    float v = (midi - 24.0f) / 36.0f;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

inline float DriftHzToNorm(float hz)
{
    constexpr float lo = 0.01f, hi = 1.0f;
    if (hz < lo) hz = lo;
    if (hz > hi) hz = hi;
    return std::log(hz / lo) / std::log(hi / lo);
}

inline float FogHzToNorm(float hz)
{
    constexpr float lo = 200.0f, hi = 8000.0f;
    if (hz < lo) hz = lo;
    if (hz > hi) hz = hi;
    return std::log(hz / lo) / std::log(hi / lo);
}

inline float BandCenterNorm(uint8_t band, uint8_t n_bands)
{
    return (static_cast<float>(band) + 0.5f) / static_cast<float>(n_bands);
}

/* Pot index aliases (0-based, match hardware.h ordering P1..P6). */
constexpr uint8_t P1 = 0, P2 = 1, P3 = 2, P4 = 3, P5 = 4, P6 = 5;

} // namespace

/* ─────────────────────────────────────────────────────────────────── */
/*  Init                                                               */
/* ─────────────────────────────────────────────────────────────────── */

void Controller::Init(VoigtpadDsp::Engine* engine)
{
    engine_ = engine;
    page_   = Page::One;
    defer_active_ = false;

    /* Page 1 defaults:
     *   P1 root  = MIDI 36
     *   P2 shmr  = 0.5
     *   P3 det   = 15 cents
     *   P4 main  = 0.5
     *   P5 chord = min  (band 0)
     *   P6 sub   = 0.5
     */
    pots_[0][P1].stored = MidiToNorm(36.0f);
    pots_[0][P2].stored = 0.5f;
    pots_[0][P3].stored = 15.0f / 30.0f;
    pots_[0][P4].stored = 0.5f;
    pots_[0][P5].stored = BandCenterNorm(0, kChordBands);
    pots_[0][P6].stored = 0.5f;

    /* Page 2 defaults:
     *   P1 drift  = 0.15 Hz
     *   P2 octave = 2  (band 2)
     *   P3 air    = 0.5
     *   P4 fog    = 4450 Hz
     *   P5 warmth = 0.2
     *   P6 unused = 0.5 (frozen)
     */
    pots_[1][P1].stored = DriftHzToNorm(0.15f);
    pots_[1][P2].stored = BandCenterNorm(2, kOctaveBands);
    pots_[1][P3].stored = 0.5f;
    pots_[1][P4].stored = FogHzToNorm(4450.0f);
    pots_[1][P5].stored = 0.2f;
    pots_[1][P6].stored = 0.5f;

    /* All pots come up uncaught — physical positions don't match the
     * spec defaults, so we keep the stored value frozen until the user
     * crosses through.  sign_at_lock is set on first Update().        */
    for (uint8_t pg = 0; pg < kNumPages; pg++)
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            pots_[pg][p].caught       = false;
            pots_[pg][p].sign_at_lock = 0;
            defer_dirty_[pg][p]       = false;
            defer_committed_[pg][p]   = pots_[pg][p].stored;
            defer_phys_[pg][p]        = pots_[pg][p].stored;
        }
    }

    PushParamsToEngine();
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Pot-catch (verbatim from athanor)                                  */
/* ─────────────────────────────────────────────────────────────────── */

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

    /* Initial sign-at-lock latch on first observation after page change /
     * Init().                                                          */
    if (s.sign_at_lock == 0)
    {
        s.sign_at_lock = (diff > 0.0f) ? 1 : -1;
        return;
    }

    const int cur_sign = (diff > 0.0f) ? 1 : -1;
    if (cur_sign != s.sign_at_lock)
    {
        s.caught = true;
        s.stored = phys;
        return;
    }
}

void Controller::ResetCatchForCurrentPage(const float pot_values[NUM_POTS])
{
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

/* ─────────────────────────────────────────────────────────────────── */
/*  Update                                                             */
/* ─────────────────────────────────────────────────────────────────── */

void Controller::Update(const float pot_values[NUM_POTS],
                        bool        b1_rising_edge,
                        bool        b2_pressed,
                        bool        b3_rising_edge)
{
    /* Snapshot for renderer. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        last_phys_[p] = pot_values[p];

    /* B3 — reserved.  Consume rising edge silently. */
    (void)b3_rising_edge;

    /* B1 — page advance. */
    if (b1_rising_edge)
    {
        /* If B2 is currently held and we're swapping pages, drop the
         * deferred-apply transaction without committing.  The user
         * can re-acquire on the new page if they want.                */
        if (defer_active_)
        {
            const uint8_t pg = static_cast<uint8_t>(page_);
            for (uint8_t p = 0; p < NUM_POTS; p++)
                defer_dirty_[pg][p] = false;
        }

        page_ = (page_ == Page::One) ? Page::Two : Page::One;
        ResetCatchForCurrentPage(pot_values);
    }

    /* ── Mid button: deferred-apply state machine ───────────────── */

    if (b2_pressed && !defer_active_)
    {
        /* Press: snapshot the committed values for both pages.  We
         * snapshot all pots so that if the user flips pages mid-hold
         * (we abort instead, see above) the data is still consistent. */
        defer_active_ = true;
        for (uint8_t pg = 0; pg < kNumPages; pg++)
        {
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                defer_committed_[pg][p] = pots_[pg][p].stored;
                defer_dirty_[pg][p]     = false;
                defer_phys_[pg][p]      = pots_[pg][p].stored;
            }
        }
    }
    else if (!b2_pressed && defer_active_)
    {
        /* Release: commit dirty pots on every page (typically only
         * the current page has dirties, but we sweep both to be safe).
         * The catch invariant is preserved: any pot that becomes
         * dirty is, by definition, being driven by the user, so we
         * mark it caught at the new physical value.                   */
        for (uint8_t pg = 0; pg < kNumPages; pg++)
        {
            for (uint8_t p = 0; p < NUM_POTS; p++)
            {
                if (defer_dirty_[pg][p])
                {
                    pots_[pg][p].stored       = defer_phys_[pg][p];
                    pots_[pg][p].caught       = true;
                    pots_[pg][p].sign_at_lock = 0;
                    defer_dirty_[pg][p]       = false;
                }
            }
        }
        defer_active_ = false;
    }

    if (defer_active_)
    {
        /* While held: track physical positions in a side table and
         * mark dirty when the user has moved more than the catch
         * tolerance from the snapshot.  Audio params do NOT update.   */
        const uint8_t pg = static_cast<uint8_t>(page_);
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            const float phys = pot_values[p];
            defer_phys_[pg][p] = phys;
            if (!defer_dirty_[pg][p])
            {
                if (std::fabs(phys - defer_committed_[pg][p])
                    > kCatchTolerance)
                {
                    defer_dirty_[pg][p] = true;
                }
            }
        }
        /* Skip the live UpdateCatch / PushParamsToEngine while held —
         * audio is intentionally frozen.  */
    }
    else
    {
        /* Live tracking on the active page. */
        for (uint8_t p = 0; p < NUM_POTS; p++)
            UpdateCatch(p, pot_values[p]);

        PushParamsToEngine();
    }
}

/* ─────────────────────────────────────────────────────────────────── */
/*  Push to engine                                                     */
/* ─────────────────────────────────────────────────────────────────── */

void Controller::PushParamsToEngine()
{
    if (!engine_)
        return;

    const PotState (&pg1)[NUM_POTS] = pots_[0];
    const PotState (&pg2)[NUM_POTS] = pots_[1];

    VoigtpadDsp::Params p;

    /* Page 1 */
    p.root_midi      = MidiFromNorm        (pg1[P1].stored);
    p.shimmer_level  = Linear01            (pg1[P2].stored);
    p.detune_cents   = DetuneCentsFromNorm (pg1[P3].stored);
    p.main_level     = Linear01            (pg1[P4].stored);
    p.chord_type     = static_cast<VoigtpadDsp::ChordType>(
                          BandIndex(pg1[P5].stored, kChordBands));
    p.sub_level      = Linear01            (pg1[P6].stored);

    /* Page 2 */
    p.shimmer_drift  = DriftHzFromNorm     (pg2[P1].stored);
    p.shimmer_octave = BandIndex(pg2[P2].stored, kOctaveBands);
    p.shimmer_air    = Linear01            (pg2[P3].stored);
    p.fog_cutoff_hz  = FogCutoffFromNorm   (pg2[P4].stored);
    p.sub_warmth     = Linear01            (pg2[P5].stored);
    /* pg2[P6] is reserved — currently unused. */

    engine_->SetParams(p);
}

/* ─────────────────────────────────────────────────────────────────── */
/*  LED rendering                                                      */
/* ─────────────────────────────────────────────────────────────────── */

namespace {

/* Map normalised value to clock-face hour (7:30 → 4:30 sweep). */
inline float ValueToHour(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    constexpr float kStart = 7.5f;
    constexpr float kSpan  = 9.0f;
    return kStart + v * kSpan;
}

/* Number of LEDs to fill in the 13-step arc. */
inline uint8_t ValueToFillSteps(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    constexpr uint8_t kSteps = 13;
    return static_cast<uint8_t>(v * static_cast<float>(kSteps) + 0.5f);
}

inline LedPanel::Rgb ScaleGlobal(const LedPanel::Rgb& c)
{
    return LedPanel::Scale(c, kGlobalBrightness);
}

constexpr uint8_t kArcLeds = 13;

/* Common page colour palette. */
constexpr LedPanel::Rgb kPagePurple = {0x80, 0x10, 0xFF};
constexpr LedPanel::Rgb kPageCyan   = {0x10, 0xC8, 0xFF};
constexpr LedPanel::Rgb kPipWhite   = {0xFF, 0xFF, 0xFF};
constexpr LedPanel::Rgb kArcDim     = {0x06, 0x06, 0x06};
constexpr LedPanel::Rgb kBlack      = {0x00, 0x00, 0x00};

/* Fixed 5-band palettes.  Chosen as smooth gradients that read from
 * left → right around the ring; tuned to harmonize loosely with the
 * page colours but still distinguishable when mid-button is held. */
constexpr LedPanel::Rgb kChordBandColors[5] = {
    {0xFF, 0x10, 0x40}, /* min  — magenta-red  */
    {0xFF, 0x80, 0x10}, /* maj  — amber        */
    {0xFF, 0xFF, 0x10}, /* min7 — yellow       */
    {0x40, 0xFF, 0x40}, /* sus2 — green        */
    {0x10, 0x80, 0xFF}, /* sus4 — blue         */
};
constexpr LedPanel::Rgb kOctaveBandColors[5] = {
    {0x10, 0x40, 0xFF}, /* 0 — deep blue       */
    {0x10, 0xC0, 0xFF}, /* 1 — cyan            */
    {0x40, 0xFF, 0xC0}, /* 2 — teal-mint       */
    {0xC0, 0xFF, 0x40}, /* 3 — lime            */
    {0xFF, 0xE0, 0x10}, /* 4 — gold            */
};

inline bool IsDiscretePot(uint8_t page, uint8_t pot)
{
    if (page == 0 && pot == P5) return true;  /* chord */
    if (page == 1 && pot == P2) return true;  /* shimmer octave */
    return false;
}

inline bool IsReservedPot(uint8_t page, uint8_t pot)
{
    /* Page 2 P6 reserved (not specced). */
    return (page == 1 && pot == P6);
}

inline uint8_t DiscreteBandsForPot(uint8_t page, uint8_t pot)
{
    if (page == 0 && pot == P5) return kChordBands;
    if (page == 1 && pot == P2) return kOctaveBands;
    return 1;
}

inline const LedPanel::Rgb* DiscretePaletteForPot(uint8_t page, uint8_t pot)
{
    if (page == 0 && pot == P5) return kChordBandColors;
    if (page == 1 && pot == P2) return kOctaveBandColors;
    return kChordBandColors;
}

/* Step in the 13-LED arc → band index for an N-band pot.
 * Bands are equal-width across the arc, biased to round-down so the
 * leftmost band gets steps 0..2 and the rightmost band gets the
 * residual.                                                          */
inline uint8_t ArcStepToBand(uint8_t step, uint8_t n_bands)
{
    int b = static_cast<int>(step) * static_cast<int>(n_bands)
            / static_cast<int>(kArcLeds);
    if (b < 0)              b = 0;
    if (b >= n_bands)       b = n_bands - 1;
    return static_cast<uint8_t>(b);
}

/* Render a continuous-pot arc (arc-fill + tick guides + optional
 * white pip).  `fill_value` is the value to display [0..1]; it may be
 * the stored value, the deferred physical value, etc.                 */
void RenderContinuousArc(uint8_t pot, const LedPanel::Rgb& page_col,
                         float   fill_value,
                         float   brightness_scale,
                         bool    show_pip,
                         float   pip_value,
                         const LedPanel::Rgb& pip_col = kPipWhite)
{
    const uint8_t fill = ValueToFillSteps(fill_value);
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const float    hour = 7.5f + static_cast<float>(step) * 0.75f;
        const uint8_t  off  = LedPanel::RingOffsetForHour(pot, hour);
        LedPanel::Rgb  c    = (step < fill)
                                ? LedPanel::Scale(page_col, brightness_scale)
                                : kArcDim;
        LedPanel::SetRingByOffset(pot, off, ScaleGlobal(c));
    }
    if (show_pip)
    {
        const float   pip_hour = ValueToHour(pip_value);
        const uint8_t pip_off  = LedPanel::RingOffsetForHour(pot, pip_hour);
        LedPanel::SetRingByOffset(pot, pip_off, ScaleGlobal(pip_col));
    }
}

/* Render a discrete-band pot.  The selected band is brighter; other
 * bands at lower brightness; band membership is derived from
 * `selected_value` (the stored or pending value).  Optional pip
 * marks the committed/snapshot position when uncaught/deferred.       */
void RenderDiscreteArc(uint8_t pot, uint8_t n_bands,
                       const LedPanel::Rgb* palette,
                       float selected_value,
                       float brightness_selected,
                       float brightness_others,
                       bool  show_pip,
                       float pip_value,
                       const LedPanel::Rgb& pip_col = kPipWhite)
{
    const uint8_t sel_band = static_cast<uint8_t>(
        std::min(static_cast<int>(n_bands - 1),
                 std::max(0,
                    static_cast<int>(selected_value
                                     * static_cast<float>(n_bands)))));
    for (uint8_t step = 0; step < kArcLeds; step++)
    {
        const uint8_t band = ArcStepToBand(step, n_bands);
        const float   k    = (band == sel_band) ? brightness_selected
                                                : brightness_others;
        const float   hour = 7.5f + static_cast<float>(step) * 0.75f;
        const uint8_t off  = LedPanel::RingOffsetForHour(pot, hour);
        LedPanel::SetRingByOffset(pot, off,
            ScaleGlobal(LedPanel::Scale(palette[band], k)));
    }
    if (show_pip)
    {
        const float   pip_hour = ValueToHour(pip_value);
        const uint8_t pip_off  = LedPanel::RingOffsetForHour(pot, pip_hour);
        LedPanel::SetRingByOffset(pot, pip_off, ScaleGlobal(pip_col));
    }
}

/* Breathing multiplier — slow sine-shaped pulse (period 1.5 s).      */
inline float Breath(uint32_t t_ms)
{
    constexpr float period_ms = 1500.0f;
    constexpr float kTwoPi    = 6.28318530718f;
    const float phase = static_cast<float>(t_ms % static_cast<uint32_t>(period_ms))
                      / period_ms;
    /* 0.30..1.00 envelope so even at trough the LEDs remain visible. */
    return 0.65f + 0.35f * std::sin(kTwoPi * phase);
}

} // namespace

void Controller::Render(uint32_t t_ms,
                        const float pot_values[NUM_POTS])
{
    (void)pot_values;  /* same as athanor — we tracked physics in
                          Update() and drive rendering from state. */

    const uint8_t  pg     = static_cast<uint8_t>(page_);
    const LedPanel::Rgb& pgcol = (page_ == Page::One)
                                    ? kPagePurple : kPageCyan;
    const float breath_k = Breath(t_ms);

    LedPanel::Clear();

    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        const PotState& s = pots_[pg][p];

        /* Reserved pot → all dim, no interaction. */
        if (IsReservedPot(pg, p))
        {
            for (uint8_t step = 0; step < kArcLeds; step++)
            {
                const float   hour = 7.5f + static_cast<float>(step) * 0.75f;
                const uint8_t off  = LedPanel::RingOffsetForHour(p, hour);
                LedPanel::SetRingByOffset(p, off, ScaleGlobal(kArcDim));
            }
            continue;
        }

        /* Display logic differs for deferred-apply state. */
        if (defer_active_ && defer_dirty_[pg][p])
        {
            /* Deferred + dirty: show the *physical* (pending) value
             * with breathing brightness; pip on the snapshot value
             * so the user can see what'll snap to / from on release. */
            const float k_breath = breath_k;
            if (IsDiscretePot(pg, p))
            {
                const uint8_t            n_bands = DiscreteBandsForPot(pg, p);
                const LedPanel::Rgb*     palette = DiscretePaletteForPot(pg, p);
                RenderDiscreteArc(p, n_bands, palette,
                                  defer_phys_[pg][p],
                                  /*sel*/  k_breath * 1.00f,
                                  /*oth*/  k_breath * 0.25f,
                                  /*pip*/  true,
                                  /*pip_v*/defer_committed_[pg][p]);
            }
            else
            {
                RenderContinuousArc(p, pgcol,
                                    defer_phys_[pg][p],
                                    /*scale*/ k_breath,
                                    /*pip*/   true,
                                    /*pip_v*/ defer_committed_[pg][p]);
            }
            continue;
        }

        if (IsDiscretePot(pg, p))
        {
            /* Live discrete render — selected band brighter.
             * Pip shown when uncaught (catch UI is preserved). */
            const uint8_t            n_bands = DiscreteBandsForPot(pg, p);
            const LedPanel::Rgb*     palette = DiscretePaletteForPot(pg, p);
            const float k_sel = s.caught ? 1.00f : 0.45f;
            const float k_oth = s.caught ? 0.25f : 0.15f;
            RenderDiscreteArc(p, n_bands, palette,
                              s.stored,
                              k_sel, k_oth,
                              !s.caught, s.stored);
            continue;
        }

        /* Continuous live render — same convention as athanor:
         *   uncaught → arc dimmed, white pip on stored value
         *   caught   → arc full, no pip                         */
        const float k = s.caught ? 1.0f : 0.30f;
        RenderContinuousArc(p, pgcol, s.stored, k,
                            !s.caught, s.stored);
    }

    /* Button accent LEDs:
     *   B1 — page colour at moderate brightness so the user knows
     *        which page is active (purple = page 1, cyan = page 2).
     *   B2 — neutral white when idle; breathing page-colour while
     *        deferred-apply is active.
     *   B3 — reserved (dim).                                         */
    LedPanel::Rgb b1 = LedPanel::Scale(pgcol, 0.55f);
    LedPanel::SetButtonPair(0, ScaleGlobal(b1));

    LedPanel::Rgb b2;
    if (defer_active_)
        b2 = LedPanel::Scale(pgcol, breath_k);
    else
        b2 = {0x10, 0x10, 0x10};
    LedPanel::SetButtonPair(1, ScaleGlobal(b2));

    LedPanel::Rgb dim = {0x05, 0x05, 0x05};
    LedPanel::SetButtonPair(2, ScaleGlobal(dim));
}

} // namespace VoigtpadUi
