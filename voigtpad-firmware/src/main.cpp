/**
 * @file main.cpp
 * @brief Voigtpad firmware entry point.
 *
 * Always-on stereo drone synth (port of `gas_chords.dsp`) running on
 * the Athanor panel hardware (Daisy Seed2 DFM + 6 pots × 3 buttons ×
 * 102 WS2812 LEDs).
 *
 * Audio runs in libDaisy's SAI DMA callback; the UI runs in the main
 * loop at ~60 Hz, polling buttons every 1 ms (athanor's pattern,
 * required for libDaisy's `Switch` debouncer to register short taps).
 * The UI never blocks audio; pot updates are pushed into the engine
 * via `SetParams()` and slewed at sample rate inside the engine, so
 * UI-rate updates produce zipper-free audio.
 */

#include "daisy_seed.h"
#include "hardware.h"
#include "leds.h"
#include "dsp.h"
#include "controller.h"

using namespace daisy;

/* ── Globals ──────────────────────────────────────────────────────── */

static DaisySeed             hw;
static Switch                buttons[NUM_BUTTONS];
static VoigtpadDsp::Engine   engine;
static VoigtpadUi::Controller ui;

/* ── Audio callback ──────────────────────────────────────────────── */

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    /* Drone synth — input is ignored. */
    engine.ProcessBlock(IN_L, IN_R, OUT_L, OUT_R, size);
}

/* ── Pot reader (raw → logical 0..1) ─────────────────────────────── */
/* Same convention as athanor: physical CW = increasing parameter. */
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

    /* LEDs (TIM3 + DMA). */
    LedPanel::Init();

    /* Audio engine + UI. */
    engine.Init();
    ui.Init(&engine);

    hw.SetLed(true);
    hw.StartAudio(AudioCallback);

    /* UI loop: ~60 Hz frames, 1 ms button polling within each frame.
     * B1 / B3 use rising edges (page change / reserved tap).
     * B2 reports a held state (deferred apply); we OR the held flag
     * across the frame so a brief blip during the frame still
     * registers correctly. */
    constexpr uint32_t kFrameMs      = 16;
    constexpr uint32_t kButtonPollMs = 1;
    uint32_t           frame_t       = 0;

    for (;;)
    {
        bool b1_edge   = false;
        bool b3_edge   = false;
        bool b2_held   = false;

        for (uint32_t elapsed = 0; elapsed < kFrameMs;
             elapsed += kButtonPollMs)
        {
            for (uint8_t i = 0; i < NUM_BUTTONS; i++)
                buttons[i].Debounce();

            if (buttons[0].RisingEdge()) b1_edge = true;
            if (buttons[1].Pressed())    b2_held = true;
            if (buttons[2].RisingEdge()) b3_edge = true;

            System::Delay(kButtonPollMs);
        }

        /* Snapshot pot values (logical 0..1, CW-positive). */
        float pots[NUM_POTS];
        for (uint8_t i = 0; i < NUM_POTS; i++)
            pots[i] = ReadPotLogical(i);

        ui.Update(pots, b1_edge, b2_held, b3_edge);
        ui.Render(frame_t, pots);
        LedPanel::Show();

        frame_t += kFrameMs;
    }
}
