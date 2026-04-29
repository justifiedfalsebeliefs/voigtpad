/**
 * @file controller.cpp
 * @brief V1 UI controller — see controller.h.
 */

#include "controller.h"

#include <algorithm>
#include <cmath>

namespace AthanorUi {

/* ── Pot → parameter mapping ─────────────────────────────────────── */
/*
 * Pot indices on page 1 ("Time" page):
 *   P1 (idx 0) = line 1 delay time   P2 (idx 1) = line 2 delay time
 *   P3 (idx 2) = line 1 feedback     P4 (idx 3) = line 2 feedback
 *   P5 (idx 4) = line 1 mix          P6 (idx 5) = line 2 mix
 *
 * The hardware pot wiring is such that the ADC reading is "inverted"
 * relative to the natural CW = increasing convention.  The existing
 * panel code computes the displayed angle from `(1.0 - v)`; we use the
 * same invariant here so the parameter increases when the user turns
 * the knob clockwise.  All `phys` values handed to the controller are
 * already in this normalised "logical" 0..1 frame.
 */

float Controller::DelaySecondsFromNorm(float v)
{
    /* Logarithmic taper: 1 ms → 2 s with musical resolution across
     * the full knob travel.   t = t_min · (t_max/t_min)^v.            */
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    constexpr float kMin = 0.001f;
    constexpr float kMax = 2.000f;
    return kMin * std::pow(kMax / kMin, v);
}

float Controller::FeedbackFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    /* 0 .. 110 % per spec. */
    return v * 1.10f;
}

float Controller::MixFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

/* Page-2 ("Space") mappers.  All three controls present a 0..1 surface
 * to the user; the engine handles the perceptual taper internally
 * (g = 0.7·v for diffusion, exp(−5·v) for damping, linear for bloom).
 * Keeping these as pass-throughs makes the UI ↔ engine coupling
 * trivial to reason about and lets the catch state act on the same
 * values that drive the audio. */
float Controller::DiffusionFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

float Controller::DampingFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

float Controller::BloomFromNorm(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return v;
}

/* ── Lifecycle ───────────────────────────────────────────────────── */

void Controller::Init(AthanorDsp::DualDelay* engine)
{
    engine_   = engine;
    page_     = Page::Time;

    /* Page-1 defaults: musical delay (300 ms), 30% feedback, 50% mix. */
    constexpr float kDefaultTime     = 0.61f;   /* ≈ 300 ms after taper */
    constexpr float kDefaultFeedback = 0.30f;
    constexpr float kDefaultMix      = 0.50f;

    /* Page-2 defaults: clean delay (no diffusion / damping / bloom).
     * This preserves the V1 "clean delay" character at boot; the user
     * adds space deliberately by switching to page 2 and turning each
     * pot through its catch position.                                  */
    constexpr float kDefaultDiffusion = 0.0f;
    constexpr float kDefaultDamping   = 0.0f;
    constexpr float kDefaultBloom     = 0.0f;

    for (uint8_t pg = 0; pg < kNumPages; pg++)
    {
        for (uint8_t p = 0; p < NUM_POTS; p++)
        {
            pots_[pg][p].caught       = true;
            pots_[pg][p].sign_at_lock = 0;
            if (pg == static_cast<uint8_t>(Page::Time))
            {
                switch (p)
                {
                    case 0: case 1: pots_[pg][p].stored = kDefaultTime;     break;
                    case 2: case 3: pots_[pg][p].stored = kDefaultFeedback; break;
                    default:        pots_[pg][p].stored = kDefaultMix;      break;
                }
            }
            else /* Page::Space */
            {
                switch (p)
                {
                    case 0: case 1: pots_[pg][p].stored = kDefaultDiffusion; break;
                    case 2: case 3: pots_[pg][p].stored = kDefaultDamping;   break;
                    default:        pots_[pg][p].stored = kDefaultBloom;     break;
                }
            }
        }
    }

    PushParamsToEngine();
}

/* ── Pot-catch ──────────────────────────────────────────────────── */

void Controller::UpdateCatch(uint8_t pot_idx, float phys)
{
    PotState& s = pots_[static_cast<uint8_t>(page_)][pot_idx];

    if (s.caught)
    {
        /* Already live — track the physical position.  No catch logic
         * needed.                                                     */
        s.stored = phys;
        return;
    }

    /* Within tolerance → catch immediately. */
    const float diff = phys - s.stored;
    if (std::fabs(diff) <= kCatchTolerance)
    {
        s.caught = true;
        s.stored = phys;
        return;
    }

    /* Crossed through the stored value (sign flipped) → catch. */
    const int cur_sign = (diff > 0.0f) ? 1 : -1;
    if (s.sign_at_lock != 0 && cur_sign != s.sign_at_lock)
    {
        s.caught = true;
        s.stored = phys;
        return;
    }
    /* Otherwise: pot remains uncaught, parameter frozen. */
}

void Controller::Update(const float pot_values[NUM_POTS],
                        bool        b1_rising_edge)
{
    /* Snapshot for renderer. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        last_phys_[p] = pot_values[p];

    /* Page advance — single press cycles 1 → 2 → 1. */
    if (b1_rising_edge)
    {
        const Page next = (page_ == Page::Time) ? Page::Space : Page::Time;
        page_ = next;

        /* Reset pot-catch state for the new page based on where the
         * physical pots currently sit.                                */
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
                /* `stored` is left untouched; this is the value the
                 * user must rotate to in order to "catch" the pot.   */
            }
        }
    }

    /* Per-pot catch + tracking on the active page. */
    for (uint8_t p = 0; p < NUM_POTS; p++)
        UpdateCatch(p, pot_values[p]);

    /* Push live page-1 values into the audio engine on every update.
     * The engine slews internally so this is safe / zipper-free.      */
    PushParamsToEngine();
}

void Controller::PushParamsToEngine()
{
    if (!engine_)
        return;

    /* Page-1 stored values map to Time/Feedback/Mix; page-2 stored
     * values map to Diffusion/Damping/Bloom.  Both pages always
     * push (the stored values track the physical pots once caught,
     * and stay frozen at the stored value until then), so switching
     * pages does not interrupt or zero any audio parameter.          */
    const PotState (&pg1)[NUM_POTS] = pots_[static_cast<uint8_t>(Page::Time)];
    const PotState (&pg2)[NUM_POTS] = pots_[static_cast<uint8_t>(Page::Space)];

    /* Line 1: P1=time, P3=fb, P5=mix; P1=diff, P3=damp, P5=bloom. */
    engine_->SetLineParams(0,
                           DelaySecondsFromNorm(pg1[0].stored),
                           FeedbackFromNorm   (pg1[2].stored),
                           MixFromNorm        (pg1[4].stored),
                           DiffusionFromNorm  (pg2[0].stored),
                           DampingFromNorm    (pg2[2].stored),
                           BloomFromNorm      (pg2[4].stored));

    /* Line 2: P2=time, P4=fb, P6=mix; P2=diff, P4=damp, P6=bloom. */
    engine_->SetLineParams(1,
                           DelaySecondsFromNorm(pg1[1].stored),
                           FeedbackFromNorm   (pg1[3].stored),
                           MixFromNorm        (pg1[5].stored),
                           DiffusionFromNorm  (pg2[1].stored),
                           DampingFromNorm    (pg2[3].stored),
                           BloomFromNorm      (pg2[5].stored));
}

/* ── LED rendering ──────────────────────────────────────────────── */
/*
 * Page colour profiles:
 *   Page 1 (Time)  → green  (#10B040)
 *   Page 2 (Space) → red    (#FF2010)
 * White pip on the LED nearest the stored value when the pot is
 * uncaught; otherwise a solid filled-arc value indicator.
 */

namespace {

/* Map a normalised 0..1 value to a clock-face hour, sweeping
 * 7:30 CW to 4:30 (270° of arc — 13 LEDs at 0.75h spacing, i.e.
 * the full 16-LED ring minus the three positions adjacent to 6
 * o'clock).  The span is an exact multiple of the 0.75h LED pitch
 * so that every step falls on a real ring LED. */
inline float ValueToHour(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    constexpr float kStart = 7.5f;
    constexpr float kSpan  = 9.0f;   /* 12 steps * 0.75h */
    return kStart + v * kSpan;
}

/* Convert a 0..1 normalised value to the number of LEDs to light in
 * the 7:30→4:30 o'clock arc.  Always at least 1 once v > 0 so there's
 * a "starting" LED visible.                                          */
inline uint8_t ValueToFillSteps(float v)
{
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    /* The arc consists of the 13 ring LEDs that are not adjacent to
     * 6 o'clock (16 total − 3 bottom).  At 0.75h pitch they span 9
     * hours (270°). */
    constexpr uint8_t kSteps = 13;
    const uint8_t lit = static_cast<uint8_t>(v * static_cast<float>(kSteps) + 0.5f);
    return lit;
}

inline LedPanel::Rgb ScaleGlobal(const LedPanel::Rgb& c)
{
    return LedPanel::Scale(c, kGlobalBrightness);
}

} // namespace

void Controller::Render(uint32_t t_ms,
                        const float pot_values[NUM_POTS])
{
    /* Δt for animation phases. */
    const float dt_ms = (last_t_ms_ == 0)
                            ? 16.0f
                            : static_cast<float>(t_ms - last_t_ms_);
    last_t_ms_ = t_ms;

    /* Page colours. */
    const LedPanel::Rgb kPageGreen = {0x18, 0xC0, 0x40};
    const LedPanel::Rgb kPageRed   = {0xE0, 0x18, 0x10};
    const LedPanel::Rgb kPipWhite  = {0xFF, 0xFF, 0xFF};
    const LedPanel::Rgb kArcDim    = {0x06, 0x06, 0x06};

    const bool                pg_time = (page_ == Page::Time);
    const LedPanel::Rgb       pgcol   = pg_time ? kPageGreen : kPageRed;

    /* Update bottom-LED flash phases (delay-time pots P1, P2 on
     * page 1 only).
     *
     * The flash visually indicates the delay period like a tap-tempo
     * blink.  As the delay shortens, the flash rate increases until
     * individual blinks blur into one another; past that point we
     * hold the LED solid and tint it more red as the delay continues
     * to shorten, giving a clear "very fast / nearly comb-filter"
     * cue without strobing.
     *
     * Rules:
     *   • Above the fusion threshold (period ≥ kFlashSolidPeriodMs):
     *     phase-accumulate the real delay period and render a fixed-
     *     width on-pulse so each blip looks identical at any speed.
     *   • Below it: stop animating the phase (the LED is solid).
     *
     * The fusion threshold sits well above the 30 Hz Nyquist limit of
     * a 60 Hz UI (period ≈ 33 ms), so the flash remains crisp right
     * up to the cutover.
     */
    constexpr float kFlashSolidPeriodMs = 60.0f; /* ≈16.7 Hz fusion point */
    if (pg_time)
    {
        for (uint8_t i = 0; i < 2; i++)
        {
            const float period_ms =
                DelaySecondsFromNorm(pots_[0][i].stored) * 1000.0f;

            if (period_ms >= kFlashSolidPeriodMs)
            {
                flash_phase_ms_[i] += dt_ms;
                if (flash_phase_ms_[i] >= period_ms)
                    flash_phase_ms_[i] =
                        std::fmod(flash_phase_ms_[i], period_ms);
            }
            else
            {
                /* Solid mode — freeze phase so we resume cleanly when
                 * the user slows the delay back down past the fusion
                 * threshold. */
                flash_phase_ms_[i] = 0.0f;
            }
        }
    }
    else
    {
        flash_phase_ms_[0] = flash_phase_ms_[1] = 0.0f;
    }

    LedPanel::Clear();

    for (uint8_t p = 0; p < NUM_POTS; p++)
    {
        const PotState& s    = pots_[static_cast<uint8_t>(page_)][p];
        (void)pot_values;  /* phys not needed in render — pot tracking
                              already happened in Update().            */

        /* The arc represents the *stored* value in both states.  When
         * the pot is uncaught the parameter is frozen at `stored`
         * regardless of physical position, and once it catches, the
         * physical position is mirrored into `stored` immediately.
         * The visual difference between the two states is conveyed by
         * the brightness scale and the white pip below.               */
        const uint8_t fill   = ValueToFillSteps(s.stored);

        /* Walk the 7:30 → 4:30 o'clock arc, lighting LEDs up to `fill`.
         * Stepping in 0.75h increments (the physical LED pitch) keeps
         * each step exactly on a real ring LED.  The arc covers the
         * 13 LEDs that are not adjacent to 6 o'clock. */
        constexpr uint8_t kArcLeds = 13;
        for (uint8_t step = 0; step < kArcLeds; step++)
        {
            const float hour = 7.5f + static_cast<float>(step) * 0.75f;
            const uint8_t off = LedPanel::RingOffsetForHour(p, hour);

            LedPanel::Rgb c;
            if (step < fill)
            {
                /* Filled portion of the arc.  Dim the value display
                 * when the pot is uncaught, so the pip is unambiguous. */
                const float k = s.caught ? 1.0f : 0.30f;
                c = LedPanel::Scale(pgcol, k);
            }
            else
            {
                /* Outline tick — barely visible scale guide. */
                c = kArcDim;
            }
            LedPanel::SetRingByOffset(p, off, ScaleGlobal(c));
        }

        /* White pip on the stored position whenever the pot is
         * uncaught.  Drawn last so it overrides the arc.             */
        if (!s.caught)
        {
            const float   pip_hour = ValueToHour(s.stored);
            const uint8_t pip_off  = LedPanel::RingOffsetForHour(p, pip_hour);
            LedPanel::SetRingByOffset(p, pip_off, ScaleGlobal(kPipWhite));
        }
    }

    /* Bottom-LED indicator on the delay-time pots (P1, P2) on page 1.
     *
     * The bottom LED of a pot ring lives nearest 6 o'clock.  Behavior:
     *
     *   • Slow / medium delays (period ≥ kFlashSolidPeriodMs):
     *     blink white with a fixed-width on-pulse so every blip looks
     *     identical at any tempo.  The on-pulse width is also capped
     *     to a fraction of the period so we never approach 100 %
     *     duty as we near the fusion threshold.
     *   • Fast delays (period < kFlashSolidPeriodMs): hold solid and
     *     tint from white toward red as the period continues to drop
     *     toward the 1 ms minimum, giving a clear visual cue that the
     *     delay is now in chorus/comb-filter territory.
     */
    if (pg_time)
    {
        constexpr float kFlashOnMs = 30.0f;
        constexpr float kMinDelayMs = 1.0f; /* matches DelaySecondsFromNorm */
        const uint8_t   time_pots[2] = {0, 1};
        for (uint8_t i = 0; i < 2; i++)
        {
            const uint8_t p      = time_pots[i];
            const uint8_t bottom = LedPanel::RingOffsetForHour(p, 6.0f);
            const float   period_ms =
                DelaySecondsFromNorm(pots_[0][i].stored) * 1000.0f;

            if (period_ms >= kFlashSolidPeriodMs)
            {
                /* Flash mode.  Cap the on-pulse to half the period so
                 * the off portion is always clearly visible right up
                 * to the cutover. */
                const float on_ms = std::min(kFlashOnMs, period_ms * 0.5f);
                if (flash_phase_ms_[i] < on_ms)
                {
                    LedPanel::SetRingByOffset(p, bottom,
                                              ScaleGlobal({0xFF, 0xFF, 0xFF}));
                }
            }
            else
            {
                /* Solid mode — mix white → red as the period shrinks
                 * from the fusion threshold down to the minimum. */
                const float t = std::max(
                    0.0f,
                    std::min((kFlashSolidPeriodMs - period_ms)
                                 / (kFlashSolidPeriodMs - kMinDelayMs),
                             1.0f));
                const LedPanel::Rgb white = {0xFF, 0xFF, 0xFF};
                const LedPanel::Rgb red   = {0xFF, 0x00, 0x00};
                const LedPanel::Rgb c     = LedPanel::Mix(white, red, t);
                LedPanel::SetRingByOffset(p, bottom, ScaleGlobal(c));
            }
        }
    }

    /* Button accent LEDs:
     *   B1 (page button) — page colour, dim/lit so the user knows
     *                      which page is active.
     *   B2/B3            — very dim "reserved" indicator.            */
    LedPanel::Rgb b1col = LedPanel::Scale(pgcol, 0.55f);
    LedPanel::SetButtonPair(0, ScaleGlobal(b1col));
    LedPanel::Rgb dim   = {0x05, 0x05, 0x05};
    LedPanel::SetButtonPair(1, ScaleGlobal(dim));
    LedPanel::SetButtonPair(2, ScaleGlobal(dim));
}

} // namespace AthanorUi
