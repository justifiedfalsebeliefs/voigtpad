/**
 * @file main.cpp
 * @brief Athanor V1 firmware entry point.
 *
 * V1 is a single-mode (series stereo), two-page, audio-only delay:
 *   • Stereo in → Delay 1 → Delay 2 → stereo out (per channel).
 *   • Page 1 (Time) controls delay time / feedback / mix per line.
 *   • Page 2 (Space) controls diffusion / damping / bloom per line.
 *   • B1 cycles pages.  B2 / B3 inactive in V1.
 *
 * See `docs/dualdelayreqs.md` and `src/dsp.h` / `src/controller.h`
 * for the full design.
 */

#include "daisy_seed.h"
#include "hardware.h"
#include "leds.h"
#include "dsp.h"
#include "controller.h"

#include <atomic>
#include <cmath>

using namespace daisy;

/* ── Global hardware ─────────────────────────────────────────────── */

static DaisySeed              hw;
static Switch                 buttons[NUM_BUTTONS];
static AthanorDsp::DualDelay  engine;
static AthanorUi::Controller  ui;

/* ── Audio callback ──────────────────────────────────────────────── */
/*
 * libDaisy provides AudioHandle::InputBuffer / OutputBuffer as
 * float*const* style channel pointers; IN_L/IN_R/OUT_L/OUT_R macros
 * are defined in libDaisy headers.  All processing for V1 happens in
 * AthanorDsp::DualDelay, which itself runs sample-by-sample for
 * smoothing and delay-tap interpolation.
 */
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    engine.ProcessBlock(IN_L, IN_R, OUT_L, OUT_R, size);
}

/* ── Pot reader (raw → logical 0..1) ─────────────────────────────── */
/*
 * The hardware pots are wired such that their raw ADC reading runs
 * 1.0 (full CCW) → 0.0 (full CW).  The existing panel code treats
 * (1.0 − v) as the "logical" parameter value; we keep that convention
 * — i.e. CW rotation increases the parameter.
 */
static inline float ReadPotLogical(uint8_t idx)
{
    return 1.0f - hw.adc.GetFloat(idx);
}

/* ── Main ────────────────────────────────────────────────────────── */

int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(ENGINE_BLOCK_SAMPLES);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    /* ADC: one channel per pot. */
    AdcChannelConfig adc_cfg[NUM_POTS];
    for (uint8_t i = 0; i < NUM_POTS; i++)
        adc_cfg[i].InitSingle(POT_PINS[i]);
    hw.adc.Init(adc_cfg, NUM_POTS);
    hw.adc.Start();

    /* Buttons. */
    for (uint8_t i = 0; i < NUM_BUTTONS; i++)
        buttons[i].Init(BTN_PINS[i]);

    /* LEDs. */
    LedPanel::Init();

    /* Audio engine + UI. */
    engine.Init();
    ui.Init(&engine);

    hw.SetLed(true);
    hw.StartAudio(AudioCallback);

    /* UI loop @ ~60 Hz render rate.  Audio runs in its own DMA-driven
     * callback.
     *
     * Buttons are polled an order of magnitude faster than the render
     * frame: libDaisy's Switch debouncer needs several consecutive
     * Debounce() samples before it reports a rising edge, so polling
     * only once per 16 ms render frame can make short / casual taps
     * feel laggy or be missed entirely.  We poll buttons every 1 ms
     * and accumulate the rising edge across the frame, then service
     * pots / LEDs once per frame as before.                            */
    constexpr uint32_t kFrameMs       = 16;
    constexpr uint32_t kButtonPollMs  = 1;
    uint32_t           frame_t        = 0;

    for (;;)
    {
        /* Sample buttons fast across one render frame; OR-accumulate
         * any rising edge so a press anywhere in the frame is
         * delivered to the UI on the next Update().                   */
        bool b1_edge = false;
        for (uint32_t elapsed = 0; elapsed < kFrameMs;
             elapsed += kButtonPollMs)
        {
            for (uint8_t i = 0; i < NUM_BUTTONS; i++)
                buttons[i].Debounce();

            /* B2 and B3 rising edges intentionally not consumed in V1
             * — they are reserved for tap / freeze in later versions. */
            if (buttons[0].RisingEdge())
                b1_edge = true;

            System::Delay(kButtonPollMs);
        }

        /* Snapshot pot values (logical 0..1, CW-positive). */
        float pots[NUM_POTS];
        for (uint8_t i = 0; i < NUM_POTS; i++)
            pots[i] = ReadPotLogical(i);

        ui.Update(pots, b1_edge);
        ui.Render(frame_t, pots);
        LedPanel::Show();

        frame_t += kFrameMs;
    }
}
