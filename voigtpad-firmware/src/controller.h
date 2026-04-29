/**
 * @file controller.h
 * @brief Voigtpad UI controller — pages, pot-catch, deferred apply,
 *        LED rendering.
 *
 * Layout — same hardware as athanor.  Pots in a 3×2 grid:
 *
 *     P1 (idx 0)   P2 (idx 1)
 *     P3 (idx 2)   P4 (idx 3)
 *     P5 (idx 4)   P6 (idx 5)
 *
 *  Three buttons:
 *     B1 (top, idx 0) — cycle pages 1 ↔ 2 (purple ↔ cyan)
 *     B2 (mid, idx 1) — *deferred apply*: while held, parameter
 *                       updates are suspended; on release, whatever
 *                       the pots are reading at that instant is
 *                       committed.  The active pot's arc breathes to
 *                       indicate the held / pending state.
 *     B3 (bot, idx 2) — reserved.
 *
 * Pages
 * -----
 *   Page 1 (purple): root, shimmer level, detune, main level,
 *                    chord, sub level
 *   Page 2 (cyan):   shimmer drift, shimmer octave, shimmer air,
 *                    fog cutoff, sub warmth, (P6 unused)
 *
 * Pot-catch — same semantics as athanor: when the active page changes,
 * pots whose physical position differs from the stored value freeze
 * the parameter and require either a tolerance match or a sign-flip
 * crossing to re-engage.  See controller.cpp::UpdateCatch.
 *
 * Deferred apply (B2) is *orthogonal* to pot-catch: while B2 is held,
 * `PushParamsToEngine()` is simply not invoked, so the engine sees the
 * last-committed targets and the pot tracking still updates `stored`
 * locally for visual feedback.  On B2 release, params are pushed
 * once.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstdint>
#include "hardware.h"
#include "dsp.h"
#include "leds.h"

namespace VoigtpadUi {

constexpr uint8_t kNumPages = 2;

/* Pot-catch tolerance — same value as athanor V1. */
constexpr float kCatchTolerance = 0.005f;

/* Global brightness scale to match athanor's panel feel. */
constexpr float kGlobalBrightness = 0.75f;

/* Which page-1 / page-2 pots represent discrete-option parameters.
 * Each entry is the pot index on that page; the discrete option count
 * is stored alongside.                                                */
struct DiscretePot
{
    int8_t  pot_idx;          /* -1 == none */
    uint8_t num_options;      /* number of discrete bands              */
};

constexpr DiscretePot kPage1Discrete = { 4, 5 }; /* P5 = chord (5 options)  */
constexpr DiscretePot kPage2Discrete = { 1, 5 }; /* P2 = shimmer oct (0..4) */

struct PotState
{
    float stored      = 0.5f;
    bool  caught      = true;
    int   sign_at_lock= 0;
};

enum class Page : uint8_t
{
    One = 0,   /* purple */
    Two = 1,   /* cyan   */
};

class Controller
{
  public:
    void Init(VoigtpadDsp::GasChords* engine);

    /** Read pots / buttons and update audio parameters.  `b1_rising_edge`
     *  is the page-cycle press accumulated by main.cpp; `b2_held` is
     *  the live debounced state of the mid button (true while pressed). */
    void Update(const float pot_values[NUM_POTS],
                bool        b1_rising_edge,
                bool        b2_held);

    /** Render the LED panel for the current frame. */
    void Render(uint32_t t_ms,
                const float pot_values[NUM_POTS]);

  private:
    VoigtpadDsp::GasChords* engine_   = nullptr;
    Page                    page_     = Page::One;

    /* [page][pot] */
    PotState                pots_[kNumPages][NUM_POTS];

    /* Deferred-apply state.  `defer_active_` mirrors B2's held state
     * and gates engine pushes — the engine's parameter slewing
     * absorbs whatever value is committed at the moment of release. */
    bool                    defer_active_         = false;

    /* Last physical pot reading (kept for renderer). */
    float                   last_phys_[NUM_POTS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

    /* ── Render-side animation state ─────────────────────────────
     * All animation state lives on the UI thread (Render is the only
     * writer), so no synchronisation is needed.                    */
    uint32_t                last_render_ms_           = 0;
    uint32_t                sparkle_rng_              = 0xCAFEBABEu;
    /* Sparkle intensity per arc step (0..1) — ages out per frame. */
    float                   sparkle_intensity_[13]    = {0.0f};
    /* Smoothed displayed levels for the three V/U meters
     * (main / shimmer / sub).  An exponential follower applied here
     * on top of the engine's per-block envelope removes single-frame
     * flicker without blunting the response.  Order matches the
     * audio band, not the pot index.                                */
    float                   vu_disp_main_             = 0.0f;
    float                   vu_disp_shmr_             = 0.0f;
    float                   vu_disp_sub_              = 0.0f;

    /* Push the current stored values into the audio engine. */
    void PushParamsToEngine();

    /* Update pot-catch state for one pot on the current page. */
    void UpdateCatch(uint8_t pot_idx, float phys);

    /* Per-pot draw helpers — one per animation type.  All operate
     * on the panel's WS2812 buffer; LedPanel::Show() is called by
     * the caller after a full panel paint.                         */
    void DrawArc        (uint8_t pot, float value, LedPanel::Rgb color,
                         float k_pot);
    void DrawWarmthArc  (uint8_t pot, float value, float k_pot);
    void DrawVu         (uint8_t pot, float level, float k_pot);
    void DrawChord      (uint8_t pot, uint8_t selected, float k_pot);
    void DrawOctave     (uint8_t pot, uint8_t selected, float k_pot);
    void DrawDriftPip   (uint8_t pot, float drift_value, float k_pot);
    void DrawSparkle    (uint8_t pot, float air_param,
                         uint32_t dt_ms, float k_pot);

    /* Page-1 mappers (norm 0..1 → engine units). */
    static float RootMidiFromNorm(float v);
    static float DetuneCentsFromNorm(float v);
    static uint8_t ChordFromNorm(float v);

    /* Page-2 mappers. */
    static float ShimmerDriftFromNorm(float v);
    static uint8_t ShimmerOctaveFromNorm(float v);
    static float FogCutoffFromNorm(float v);
    static float NormFromFogCutoff(float hz);
    static float FogModDepthFromNorm(float v);
};

} // namespace VoigtpadUi

#endif /* CONTROLLER_H */
