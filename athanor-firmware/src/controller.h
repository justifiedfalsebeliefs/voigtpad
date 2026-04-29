/**
 * @file controller.h
 * @brief V1 UI controller: pages, pot-catch, parameter mapping.
 *
 * Layout
 * ------
 *   Pots are arranged as two stacks of three (one stack per delay line):
 *     P1 (time₁)   P2 (time₂)
 *     P3 (fb₁)     P4 (fb₂)
 *     P5 (mix₁)    P6 (mix₂)
 *
 *   Three buttons: B1 cycles pages 1 → 2 → 1.  B2 and B3 are inactive
 *   in V1 and reserved for tap / freeze in later versions.
 *
 * Pages
 * -----
 *   Page 1 (Time / green) — Delay time, Feedback, Mix         per line
 *   Page 2 (Space / red)  — Diffusion, Damping, Bloom         per line
 *
 * Pot-catch
 * ---------
 *   Each pot has a remembered value per page.  When the active page
 *   changes, the physical pot position no longer matches the stored
 *   value, so the pot becomes "uncaught": the parameter freezes at the
 *   stored value, and the ring shows a white "pip" at the stored
 *   position.  As the user rotates the pot it is captured the moment
 *   either:
 *
 *     • |phys − stored| ≤ tolerance, OR
 *     • the user crosses through the stored value (sign of (phys-stored)
 *       has flipped relative to where they started).
 *
 *   Once caught, the white pip disappears and the pot becomes live —
 *   the stored value tracks the physical position from then on.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstdint>
#include "hardware.h"
#include "dsp.h"
#include "leds.h"

namespace AthanorUi {

/* Two pages in V1 (Time / Space). */
constexpr uint8_t kNumPages = 2;

/* Pot-catch tolerance: how close (0..1) physical must be to stored to
 * be considered "caught" without crossing.  Kept tight so the user has
 * to land precisely on the stored value — anything looser produces
 * audible parameter jumps as soon as the pot is touched.             */
constexpr float kCatchTolerance = 0.005f;

/* Global brightness scale ("reduce to 75% of what it currently is"). */
constexpr float kGlobalBrightness = 0.75f;

/* Per-pot, per-page state. */
struct PotState
{
    float stored      = 0.5f;   /* normalised 0..1 stored parameter */
    bool  caught      = true;   /* true → pot is live and tracking */
    int   sign_at_lock= 0;      /* sign of (phys - stored) when uncaught */
};

/* Logical page identifier. */
enum class Page : uint8_t
{
    Time  = 0,
    Space = 1,
};

class Controller
{
  public:
    /** Bind the controller to the audio engine. */
    void Init(AthanorDsp::DualDelay* engine);

    /** Read pots / buttons and update audio parameters.  Called from
     *  the UI loop every frame (~60 Hz).                               */
    void Update(const float pot_values[NUM_POTS],
                bool        b1_rising_edge);

    /** Render the LED panel for the current frame.                     */
    void Render(uint32_t t_ms,
                const float pot_values[NUM_POTS]);

  private:
    AthanorDsp::DualDelay* engine_   = nullptr;
    Page                   page_     = Page::Time;

    /* [page][pot] */
    PotState               pots_[kNumPages][NUM_POTS];

    /* Most recent physical pot reading, used by Render after Update. */
    float                  last_phys_[NUM_POTS] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

    /* Animation counter for the bottom-LED delay-time flash.
     * Tracked in milliseconds since the last "tick"; wraps once per
     * (clamped) delay period.  See Render() for the clamp.           */
    float                  flash_phase_ms_[2] = {0.0f, 0.0f};
    uint32_t               last_t_ms_      = 0;

    /* Push the live page-1 stored values into the audio engine. */
    void PushParamsToEngine();

    /* Update pot-catch state for one pot on the current page. */
    void UpdateCatch(uint8_t pot_idx, float phys);

    /* Map 0..1 pot value to a parameter for a given pot index on
     * page 1.                                                       */
    static float DelaySecondsFromNorm(float v);
    static float FeedbackFromNorm(float v);
    static float MixFromNorm(float v);

    /* Page-2 mappers (all return 0..1; the engine clamps).           */
    static float DiffusionFromNorm(float v);
    static float DampingFromNorm(float v);
    static float BloomFromNorm(float v);
};

} // namespace AthanorUi

#endif /* CONTROLLER_H */
