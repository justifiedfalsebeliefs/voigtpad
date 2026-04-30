/**
 * @file main.cpp
 * @brief Voigtpad firmware entry point.
 *
 * Always-droning ambient texture generator — no audio inputs are
 * consumed.  All synthesis runs in `VoigtpadDsp::GasChords` (a
 * hand-translation of `docs/gas_chords.dsp`); the UI runs in
 * `VoigtpadUi::Controller`.
 *
 * Architecture
 * ------------
 *  • Audio: SAI/I2S DMA-driven block callback (libDaisy).  Block size
 *    24 samples @ 48 kHz so the worst-case audio deadline is ~0.5 ms.
 *  • Pots: ADC continuous-conversion + DMA, polled every UI frame.
 *  • Buttons: GPIO with libDaisy's debouncer; polled at 1 kHz so brief
 *    edges are never missed.
 *  • LEDs: TIM3 + DMA WS2812 driver, painted once per UI frame.
 *
 * The audio callback only touches the engine — it never reads UI
 * state directly.  All UI → DSP traffic flows through the engine's
 * setter methods, which write parameter *targets* that the audio
 * thread slews internally; this keeps the cross-thread interaction
 * free of locks while remaining glitch-safe.
 */

#include "daisy_seed.h"
#include "hardware.h"
#include "leds.h"
#include "dsp.h"
#include "controller.h"

using namespace daisy;

/* ── Global hardware ─────────────────────────────────────────────── */

static DaisySeed                hw;
static Switch                   buttons[NUM_BUTTONS];
static VoigtpadDsp::GasChords   engine;
static VoigtpadUi::Controller   ui;

/* ── Audio callback ──────────────────────────────────────────────── */
/*
 * Drone-only — the input buffers are unused.  We write directly into
 * OUT_L / OUT_R from the engine.
 */
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    (void)in;
    engine.ProcessBlock(OUT_L, OUT_R, size);
}

/* ── Pot reader (raw → logical 0..1) ─────────────────────────────── */
/*
 * Same convention as athanor V1: ADC reading runs 1.0 (full CCW) →
 * 0.0 (full CW).  We invert so the parameter increases when turning
 * the knob clockwise.                                                 */
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

    /* ADC: one channel per pot, started in DMA continuous mode by
     * libDaisy's adc.Start().                                        */
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

    /*
     * UI loop @ ~60 Hz render; buttons polled every 1 ms so libDaisy's
     * debouncer (which needs several consecutive samples to register
     * an edge) never misses a short tap.
     *
     * B2 is reported as a *held* flag rather than an edge: the
     * controller's deferred-apply state machine needs to observe the
     * release.  We OR-accumulate B1's rising edge across the frame
     * the same way athanor does.
     */
    constexpr uint32_t kFrameMs       = 16;
    constexpr uint32_t kButtonPollMs  = 1;
    uint32_t           frame_t        = 0;

    for (;;)
    {
        bool b1_edge = false;
        bool b2_held = false;

        for (uint32_t elapsed = 0; elapsed < kFrameMs;
             elapsed += kButtonPollMs)
        {
            for (uint8_t i = 0; i < NUM_BUTTONS; i++)
                buttons[i].Debounce();

            if (buttons[0].RisingEdge())
                b1_edge = true;

            /* Sample B2 as a held state — true if the button was
             * pressed at *any* point during the frame so brief
             * releases don't accidentally commit deferred values. */
            if (buttons[1].Pressed())
                b2_held = true;

            System::Delay(kButtonPollMs);
        }

        /* Snapshot pot values (logical 0..1, CW-positive). */
        float pots[NUM_POTS];
        for (uint8_t i = 0; i < NUM_POTS; i++)
            pots[i] = ReadPotLogical(i);

        ui.Update(pots, b1_edge, b2_held);
        ui.Render(frame_t, pots);
        LedPanel::Show();

        frame_t += kFrameMs;
    }
}
