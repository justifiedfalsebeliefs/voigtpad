/**
 * @file hardware.h
 * @brief Daisy Seed2 DFM hardware pin mapping for the Athanor module.
 *
 * Hardware Revision: New panel layout (2025-04 revision)
 *   • 6 potentiometers in a 3 × 2 grid (P1..P6)
 *   • 3 buttons interleaved between pot rows (B1, B2, B3)
 *   • 102 WS2812-class addressable LEDs in a single chain:
 *       - 16 LEDs per pot ring  (96 total)
 *       - 2  LEDs per button     (6 total — one above + one below)
 *
 * Daisy Seed2 DFM pin reference (Daisy_Seed2_DFM.pdf, Tables 2-3):
 *
 *   Header C (analog):
 *     C1  = D16/A1  PA3   ADC1      C6  = D21/A6  PC4   ADC6
 *     C2  = D19/A4  PA6   ADC4      C7  = D15/A0  PC0   ADC0
 *     C3  = D20/A5  PC1   ADC5      C8  = D23/A8  PA4   DAC1/ADC8
 *     C4  = D18/A3  PA7   ADC3      C9  = D22/A7  PA5   DAC2/ADC7
 *     C5  = D17/A2  PB1   ADC2      C10 = D31/A12 PC2   ADC12
 *
 *   Header D (audio + digital):
 *     D1-D7 = Audio I/O              D8  = D32/A13 PC3
 *     D9  = D0  PB12                 D10 = D27     PG9
 *
 *   Header E (digital):
 *     E1 = D24/A9  PA1    E5 = D6  PC12    E8  = D3  PC9
 *     E2 = D25/A10 PA0    E6 = D5  PD2     E9  = D2  PC10
 *     E3 = D26     PD11   E7 = D4  PC8     E10 = D1  PC11
 *     E4 = D28/A11 PA2
 */

#ifndef HARDWARE_H
#define HARDWARE_H

#include "daisy_seed.h"

/* ========================================================================= */
/*  Audio                                                                     */
/* ========================================================================= */
/*
 * Stereo passthrough using the onboard PCM3060 Daisy codec only.
 *   Inputs:  D1-Audio_IN1_L, D2-Audio_IN1_R   (in[0], in[1])
 *   Outputs: Codec DAC L+/L-/R+/R-            (out[0], out[1])
 *
 * The external PCM1865 + SAI2 pipeline used by previous hardware revisions
 * is intentionally not initialised on this revision; those pins are now
 * repurposed (e.g. PA0 → button B1).
 */

/* ========================================================================= */
/*  Potentiometers (ADC inputs)                                               */
/* ========================================================================= */
/*
 * Physical layout (reading order, left → right then down):
 *   Pot 1 (top-left)      Pot 2 (top-right)
 *   Pot 3 (mid-left)      Pot 4 (mid-right)
 *   Pot 5 (bot-left)      Pot 6 (bot-right)
 *
 * Pin map (this hardware revision):
 *   P1 → ADC2  C5  PB1
 *   P2 → ADC6  C6  PC4
 *   P3 → ADC3  C4  PA7
 *   P4 → ADC5  C3  PC1
 *   P5 → ADC1  C1  PA3
 *   P6 → ADC4  C2  PA6
 *
 * Wired: pot wiper → ADC pin, pot legs → GND and 3V3A.
 */

constexpr uint8_t NUM_POTS = 6;

constexpr daisy::Pin PIN_POT1 = daisy::seed::D17; /* C5  - PB1, ADC2 */
constexpr daisy::Pin PIN_POT2 = daisy::seed::D21; /* C6  - PC4, ADC6 */
constexpr daisy::Pin PIN_POT3 = daisy::seed::D18; /* C4  - PA7, ADC3 */
constexpr daisy::Pin PIN_POT4 = daisy::seed::D20; /* C3  - PC1, ADC5 */
constexpr daisy::Pin PIN_POT5 = daisy::seed::D16; /* C1  - PA3, ADC1 */
constexpr daisy::Pin PIN_POT6 = daisy::seed::D19; /* C2  - PA6, ADC4 */

/* Ordered array — index 0 = Pot 1, … index 5 = Pot 6. */
constexpr daisy::Pin POT_PINS[NUM_POTS] = {
    PIN_POT1, PIN_POT2, PIN_POT3, PIN_POT4, PIN_POT5, PIN_POT6
};

/* ========================================================================= */
/*  Buttons (active-low, internal pull-up)                                    */
/* ========================================================================= */
/*
 * Pin map (this hardware revision):
 *   B1 → E3  PD11
 *   B2 → D9  PB12  (formerly USB_HS_ID, unused on this build)
 *   B3 → E9  PC10  (formerly SDMMC_D2 / SPI3_SCK; freed by new LED driver)
 */

constexpr uint8_t NUM_BUTTONS = 3;

constexpr daisy::Pin PIN_BTN1 = daisy::seed::D26; /* E3 - PD11 */
constexpr daisy::Pin PIN_BTN2 = daisy::seed::D0;  /* D9 - PB12 */
constexpr daisy::Pin PIN_BTN3 = daisy::seed::D2;  /* E9 - PC10 */

constexpr daisy::Pin BTN_PINS[NUM_BUTTONS] = {
    PIN_BTN1, PIN_BTN2, PIN_BTN3
};

/* ========================================================================= */
/*  WS2812 LED chain (102 LEDs)                                               */
/* ========================================================================= */
/*
 * Single-wire serial chain driven by TIM3_CH4 PWM + DMA on PC9 (E8 / D3).
 * The LED driver clocks the entire bitstream out via DMA so the CPU is free
 * for audio / DSP work.
 *
 * Chain wiring (in transmission order):
 *
 *   index  device            notes
 *   ──────────────────────────────────────────────────────────────────
 *   0..15  Pot 6 ring        CW from 9 o'clock
 *                              (chain  0 = 9, chain  4 = noon,
 *                               chain  8 = 3,  chain 12 = 6)
 *   16     B3 BOTTOM         single LED below button 3
 *   17..32 Pot 5 ring        CW from 3 o'clock
 *                              (chain 17 = 3, chain 29 = noon)
 *   33     B3 TOP            single LED above button 3
 *   34     B2 BOTTOM         single LED below button 2
 *   35..50 Pot 4 ring        CCW from 9 o'clock
 *                              (chain 35 = 9, chain 47 = noon)
 *   51     B2 TOP            single LED above button 2
 *   52..67 Pot 3 ring        CW from "one CCW step from 3 o'clock"
 *                              (chain 53 = 3, chain 65 = noon, chain 61 = 9)
 *   68     B1 BOTTOM         single LED below button 1
 *   69..84 Pot 1 ring        CW from 3 o'clock
 *                              (chain 69 = 3, chain 81 = noon)
 *   85     B1 TOP            single LED above button 1
 *   86..101 Pot 2 ring       CW from "one CW step from 9 o'clock"
 *                              (chain 89 = noon, chain 97 = 6, chain 101 = 9)
 *
 * All ring positions are equidistant (16 LEDs around 360° → 22.5° per step
 * → 0.75 hours of clock-face per step).
 */

constexpr daisy::Pin PIN_LED_DATA = daisy::seed::D3; /* E8 - PC9, TIM3_CH4 (AF2) */

constexpr uint8_t  LEDS_PER_RING       = 16;
constexpr uint8_t  NUM_LED_RINGS       = NUM_POTS;       /* 6 */
constexpr uint8_t  LEDS_PER_BUTTON     = 2;              /* top + bottom */
constexpr uint16_t LED_RING_TOTAL      = LEDS_PER_RING * NUM_LED_RINGS; /* 96 */
constexpr uint16_t LED_BUTTON_TOTAL    = LEDS_PER_BUTTON * NUM_BUTTONS; /* 6  */
constexpr uint16_t LED_TOTAL           = LED_RING_TOTAL + LED_BUTTON_TOTAL; /* 102 */

/* Hours per LED step around a ring (16 LEDs / 12 hours). */
constexpr float    LED_HOURS_PER_STEP  = 12.0f / static_cast<float>(LEDS_PER_RING); /* 0.75 */

/* ========================================================================= */
/*  Engine Constants                                                          */
/* ========================================================================= */
constexpr uint32_t ENGINE_SAMPLE_RATE_HZ = 48000;
constexpr uint16_t ENGINE_BLOCK_SAMPLES  = 24;

#endif /* HARDWARE_H */
