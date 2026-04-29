/**
 * @file controller.h
 * @brief Voigtpad UI controller — pages, pot-catch, deferred apply,
 *        discrete-band rendering, breathing animations.
 *
 * Design references (kept consistent with athanor):
 *   • Two pages, top button cycles, pot-catch on page change.
 *   • Pot-catch implementation is reused verbatim from athanor's
 *     `Controller::UpdateCatch` and `Controller::Update`.
 *   • All defaults are written to `pots_[page][p].stored` at Init()
 *     time so the engine starts from the spec-mandated boot state
 *     regardless of physical pot positions; pots come up uncaught and
 *     the user has to cross through to take over.
 *
 * New behaviours vs. athanor:
 *   • Mid button held → "deferred apply".  While held, pot motion
 *     does NOT push to the engine; on release, all dirty pots commit
 *     in one go.  LED feedback is a breathing animation on the moving
 *     pot.
 *   • Discrete-band pots (Chord, Shimmer Octave) round to 5 bands and
 *     render each band in its own hue, with the selected band slightly
 *     brightened — letting the user hold mid, hover a band, and
 *     release without sweeping through every value in between.
 */

#ifndef VOIGTPAD_CONTROLLER_H
#define VOIGTPAD_CONTROLLER_H

#include <cstdint>
#include "hardware.h"
#include "dsp.h"
#include "leds.h"

namespace VoigtpadUi {

/* Two pages (Page 1 = purple, Page 2 = cyan). */
constexpr uint8_t kNumPages = 2;

/* Pot-catch tolerance.  Same as athanor — see athanor controller.h. */
constexpr float kCatchTolerance = 0.005f;

/* Global brightness scale (panel comfort cap). */
constexpr float kGlobalBrightness = 0.75f;

/* Number of discrete bands for the two banded pots. */
constexpr uint8_t kChordBands  = static_cast<uint8_t>(VoigtpadDsp::kNumChordTypes); /* 5 */
constexpr uint8_t kOctaveBands = 5;  /* 0..4 */

/* Per-pot, per-page state.  Mirrors athanor's struct exactly. */
struct PotState
{
    float stored      = 0.5f;   /* normalised 0..1                  */
    bool  caught      = true;   /* live tracking?                   */
    int   sign_at_lock= 0;      /* sign of (phys - stored) at lock  */
};

enum class Page : uint8_t
{
    One = 0,
    Two = 1,
};

class Controller
{
  public:
    void Init(VoigtpadDsp::Engine* engine);

    /** Read pots / buttons.  Called once per UI frame.
     *  @param pot_values  Logical 0..1 (CW-positive) for all 6 pots.
     *  @param b1_rising_edge  Top button rising edge (page advance).
     *  @param b2_pressed      Mid button held state (deferred apply).
     *  @param b3_rising_edge  Bottom button rising edge (reserved).  */
    void Update(const float pot_values[NUM_POTS],
                bool        b1_rising_edge,
                bool        b2_pressed,
                bool        b3_rising_edge);

    /** Render the LED panel for the current frame.                  */
    void Render(uint32_t t_ms,
                const float pot_values[NUM_POTS]);

  private:
    VoigtpadDsp::Engine* engine_   = nullptr;
    Page                 page_     = Page::One;

    /* [page][pot] */
    PotState pots_[kNumPages][NUM_POTS];

    /* Deferred-apply state.                                          */
    bool  defer_active_              = false;
    bool  defer_dirty_[kNumPages][NUM_POTS] = {};
    float defer_committed_[kNumPages][NUM_POTS] = {}; /* snapshot at press */
    float defer_phys_[kNumPages][NUM_POTS]      = {}; /* live phys while held */

    /* Most recent physical pot reading (for renderer). */
    float last_phys_[NUM_POTS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

    /* Push live page-1/2 stored values into the audio engine. */
    void PushParamsToEngine();

    /* Pot-catch update for one pot on the current page. */
    void UpdateCatch(uint8_t pot_idx, float phys);

    /* Reset catch state for whichever page is now active, given the
     * current physical pot positions.                                */
    void ResetCatchForCurrentPage(const float pot_values[NUM_POTS]);

    /* Pot-norm → engine-unit mappers. */
    static float MidiFromNorm        (float v);   /* 24..60 */
    static float DetuneCentsFromNorm (float v);   /* 0..30  */
    static float Linear01            (float v);   /* 0..1   */
    static float DriftHzFromNorm     (float v);   /* 0.01..1.0 (log) */
    static float FogCutoffFromNorm   (float v);   /* 200..8000 (log) */
    static uint8_t BandIndex         (float v, uint8_t n_bands);
};

} // namespace VoigtpadUi

#endif /* VOIGTPAD_CONTROLLER_H */
