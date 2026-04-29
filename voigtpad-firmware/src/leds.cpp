/**
 * @file leds.cpp
 * @brief Athanor panel LED abstraction — chain layout + colour helpers.
 *
 * See leds.h for the public API.  This file owns the layout tables that
 * map (pot, ring-offset) → chain index and (pot, hour) → ring-offset
 * for the panel.  All other animation code talks to LedPanel:: and
 * never has to think about the physical chain order.
 *
 * Chain order (in WS2812 transmission order, 0-based):
 *
 *   chain  0..15  : Pot 6 ring   (CW from 9 o'clock)
 *   chain  16     : B3 BOTTOM
 *   chain 17..32  : Pot 5 ring   (CW from 3 o'clock)
 *   chain 33      : B3 TOP
 *   chain 34      : B2 BOTTOM
 *   chain 35..50  : Pot 4 ring   (CCW from 9 o'clock)
 *   chain 51      : B2 TOP
 *   chain 52..67  : Pot 3 ring   (CW from 1.5h-CCW of 3 o'clock)
 *   chain 68      : B1 BOTTOM
 *   chain 69..84  : Pot 1 ring   (CW from 3 o'clock)
 *   chain 85      : B1 TOP
 *   chain 86..101 : Pot 2 ring   (CW from 0.75h-CW of 9 o'clock)
 */

#include "leds.h"

#include <cmath>
#include <cstdlib>

namespace LedPanel {

/* ── Per-pot ring layout table ────────────────────────────────────── */

/**
 * Geometry of a single pot ring:
 *   start_chain  : chain index of ring offset 0 (0-based)
 *   start_hour   : clock-face hour position of ring offset 0
 *   step_hours   : hours added per ring offset step
 *                  (+0.75 for clockwise, -0.75 for counter-clockwise)
 */
struct RingLayout
{
    uint16_t start_chain;
    float    start_hour;
    float    step_hours;
};

/* Indexed by pot index (0 = Pot 1, … 5 = Pot 6). */
static constexpr float kStepCW  =  0.75f;
static constexpr float kStepCCW = -0.75f;

static constexpr RingLayout kRings[NUM_POTS] = {
    /* Pot 1 */ { 69, 3.0f,  kStepCW  },
    /* Pot 2 */ { 86, 9.75f, kStepCW  },
    /* Pot 3 */ { 52, 2.25f, kStepCW  },
    /* Pot 4 */ { 35, 9.0f,  kStepCCW },
    /* Pot 5 */ { 17, 3.0f,  kStepCW  },
    /* Pot 6 */ {  0, 9.0f,  kStepCW  },
};

/* ── Per-button accent LED layout (chain indices) ─────────────────── */

struct ButtonLayout
{
    uint16_t bottom_chain;
    uint16_t top_chain;
};

/* Indexed by button index (0 = B1, 1 = B2, 2 = B3). */
static constexpr ButtonLayout kButtons[NUM_BUTTONS] = {
    /* B1 */ { 68, 85 },
    /* B2 */ { 34, 51 },
    /* B3 */ { 16, 33 },
};

/* ── Backing driver ───────────────────────────────────────────────── */

static WS2812 strip_;

/* ========================================================================= */
/*  Lifecycle                                                                 */
/* ========================================================================= */

void Init()
{
    strip_.Init(PIN_LED_DATA, LED_TOTAL);
    Clear();
    Show();
}

void Clear()
{
    strip_.Clear();
}

void Show()
{
    strip_.Show();
}

bool Busy()
{
    return strip_.Busy();
}

/* ========================================================================= */
/*  Raw chain access                                                          */
/* ========================================================================= */

void SetChain(uint16_t chain_index, const Rgb& c)
{
    strip_.SetPixel(chain_index, c.r, c.g, c.b);
}

/* ========================================================================= */
/*  Pot ring access                                                           */
/* ========================================================================= */

void SetRingByOffset(uint8_t pot_idx, uint8_t ring_offset, const Rgb& c)
{
    if (pot_idx >= NUM_POTS || ring_offset >= LEDS_PER_RING)
        return;
    const uint16_t idx = kRings[pot_idx].start_chain + ring_offset;
    strip_.SetPixel(idx, c.r, c.g, c.b);
}

uint8_t RingOffsetForHour(uint8_t pot_idx, float hour)
{
    if (pot_idx >= NUM_POTS)
        return 0;

    const RingLayout& g = kRings[pot_idx];

    /* Steps along the ring from offset 0 to reach the requested hour.
     * Each step covers g.step_hours hours; we want the offset whose
     * angular position is closest to `hour`.                          */
    float steps = (hour - g.start_hour) / g.step_hours;

    /* Round to nearest integer offset, then wrap into [0, LEDS_PER_RING). */
    int rounded = static_cast<int>(std::lround(steps));
    int wrapped = rounded % LEDS_PER_RING;
    if (wrapped < 0)
        wrapped += LEDS_PER_RING;
    return static_cast<uint8_t>(wrapped);
}

void SetRingByHour(uint8_t pot_idx, float hour, const Rgb& c)
{
    SetRingByOffset(pot_idx, RingOffsetForHour(pot_idx, hour), c);
}

void ClearRing(uint8_t pot_idx)
{
    if (pot_idx >= NUM_POTS)
        return;
    const uint16_t base = kRings[pot_idx].start_chain;
    for (uint8_t i = 0; i < LEDS_PER_RING; i++)
        strip_.SetPixel(base + i, 0, 0, 0);
}

/* ========================================================================= */
/*  Button accent LEDs                                                        */
/* ========================================================================= */

void SetButton(uint8_t btn_idx, ButtonLed which, const Rgb& c)
{
    if (btn_idx >= NUM_BUTTONS)
        return;
    const uint16_t idx = (which == ButtonLed::Top)
                       ? kButtons[btn_idx].top_chain
                       : kButtons[btn_idx].bottom_chain;
    strip_.SetPixel(idx, c.r, c.g, c.b);
}

void SetButtonPair(uint8_t btn_idx, const Rgb& c)
{
    SetButton(btn_idx, ButtonLed::Bottom, c);
    SetButton(btn_idx, ButtonLed::Top,    c);
}

/* ========================================================================= */
/*  Colour helpers                                                            */
/* ========================================================================= */

Rgb Hsv(float h, float s, float v)
{
    /* Wrap hue into [0, 1). */
    h -= std::floor(h);
    if (s < 0.0f) s = 0.0f; else if (s > 1.0f) s = 1.0f;
    if (v < 0.0f) v = 0.0f; else if (v > 1.0f) v = 1.0f;

    const float    hh = h * 6.0f;
    const int      sector = static_cast<int>(hh);
    const float    f  = hh - static_cast<float>(sector);
    const float    p  = v * (1.0f - s);
    const float    q  = v * (1.0f - s * f);
    const float    t  = v * (1.0f - s * (1.0f - f));

    float r, g, b;
    switch (sector)
    {
        case 0:  r = v; g = t; b = p; break;
        case 1:  r = q; g = v; b = p; break;
        case 2:  r = p; g = v; b = t; break;
        case 3:  r = p; g = q; b = v; break;
        case 4:  r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break; /* sector 5 (or 6 from rounding) */
    }

    Rgb out;
    out.r = static_cast<uint8_t>(r * 255.0f + 0.5f);
    out.g = static_cast<uint8_t>(g * 255.0f + 0.5f);
    out.b = static_cast<uint8_t>(b * 255.0f + 0.5f);
    return out;
}

Rgb Scale(const Rgb& c, float k)
{
    if (k < 0.0f) k = 0.0f; else if (k > 1.0f) k = 1.0f;
    Rgb out;
    out.r = static_cast<uint8_t>(static_cast<float>(c.r) * k + 0.5f);
    out.g = static_cast<uint8_t>(static_cast<float>(c.g) * k + 0.5f);
    out.b = static_cast<uint8_t>(static_cast<float>(c.b) * k + 0.5f);
    return out;
}

Rgb Mix(const Rgb& a, const Rgb& b, float t)
{
    if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    const float u = 1.0f - t;
    Rgb out;
    out.r = static_cast<uint8_t>(static_cast<float>(a.r) * u + static_cast<float>(b.r) * t + 0.5f);
    out.g = static_cast<uint8_t>(static_cast<float>(a.g) * u + static_cast<float>(b.g) * t + 0.5f);
    out.b = static_cast<uint8_t>(static_cast<float>(a.b) * u + static_cast<float>(b.b) * t + 0.5f);
    return out;
}

} // namespace LedPanel
