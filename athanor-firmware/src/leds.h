/**
 * @file leds.h
 * @brief Athanor panel LED abstraction layer.
 *
 * Wraps the low-level @ref WS2812 driver with addressing helpers that
 * understand the panel's physical layout:
 *
 *   • 6 pot rings of 16 LEDs each, with per-ring orientation
 *     (start angle and direction) baked in.
 *   • 3 buttons with TOP / BOTTOM accent LEDs.
 *   • Helpers to address a ring LED by clock-position (in hours,
 *     0 = noon, 3 = 3 o'clock, 6 = 6 o'clock, 9 = 9 o'clock) so
 *     that animation code never has to know about the chain order.
 *
 * After any number of Set… calls, call Show() to push the frame to the
 * strip via DMA.  Show() returns immediately; the strip is updated in
 * the background.
 */

#ifndef LEDS_H
#define LEDS_H

#include <cstdint>
#include "hardware.h"
#include "ws2812.h"

namespace LedPanel {

/** Single 8-bit RGB triplet. */
struct Rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/** Button accent LED selector. */
enum class ButtonLed : uint8_t
{
    Bottom = 0,
    Top    = 1,
};

/**
 * Initialise the panel.  Must be called once at start-up, after
 * DaisySeed::Init() has finished.  Internally configures TIM3 + DMA on
 * PIN_LED_DATA via the WS2812 driver.
 */
void Init();

/** Turn every LED off (does not call Show). */
void Clear();

/**
 * Push the current frame to the LED strip.  Returns immediately —
 * the DMA engine continues to clock the data out in the background.
 * Subsequent calls block (briefly) until the previous transfer finishes.
 */
void Show();

/** True while a DMA transfer is still in progress. */
bool Busy();

/* ── Raw chain access ─────────────────────────────────────────────── */

/** Set any LED on the chain by its 0-based chain index. */
void SetChain(uint16_t chain_index, const Rgb& c);

/* ── Pot ring access ──────────────────────────────────────────────── */

/**
 * Set a pixel on a pot ring by its in-chain offset (0..LEDS_PER_RING-1).
 * Useful when iterating around the ring in transmission order, e.g.
 * for spinner animations that don't care about absolute orientation.
 */
void SetRingByOffset(uint8_t pot_idx, uint8_t ring_offset, const Rgb& c);

/**
 * Convert a clock-face hour position (0 = noon, 3, 6, 9, …, modulo 12)
 * to the corresponding chain offset (0..15) on the given pot ring,
 * snapping to the nearest physical LED.  Negative hours wrap correctly.
 */
uint8_t RingOffsetForHour(uint8_t pot_idx, float hour);

/**
 * Set the LED on a pot ring closest to the given clock-face hour.
 *   • 0  = noon
 *   • 3  = 3 o'clock (right)
 *   • 6  = 6 o'clock (bottom)
 *   • 9  = 9 o'clock (left)
 */
void SetRingByHour(uint8_t pot_idx, float hour, const Rgb& c);

/** Clear every LED on a single pot ring. */
void ClearRing(uint8_t pot_idx);

/* ── Button accent LEDs ───────────────────────────────────────────── */

/** Set the TOP or BOTTOM accent LED for the given button. */
void SetButton(uint8_t btn_idx, ButtonLed which, const Rgb& c);

/** Set both TOP and BOTTOM accent LEDs for the given button. */
void SetButtonPair(uint8_t btn_idx, const Rgb& c);

/* ── Colour helpers ───────────────────────────────────────────────── */

/**
 * Convert HSV (0..1) to RGB (0..255).  H wraps modulo 1.0; values
 * outside the 0..1 range for S/V are clamped.
 */
Rgb Hsv(float h, float s, float v);

/** Multiply each channel of @p c by @p k (0..1).  Saturating, no clamping needed. */
Rgb Scale(const Rgb& c, float k);

/** Linear blend of two colours: @p t in [0,1], 0 → a, 1 → b. */
Rgb Mix(const Rgb& a, const Rgb& b, float t);

} // namespace LedPanel

#endif /* LEDS_H */
