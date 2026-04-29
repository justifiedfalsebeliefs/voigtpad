/**
 * @file ws2812.h
 * @brief WS2812B/C LED strip driver — TIM3 PWM + DMA, zero CPU overhead.
 *
 * Each WS2812 bit is generated as one PWM cycle on TIM3 channel 4
 * (PC9, AF2) at 800 kHz.  DMA1 Stream 7 streams CCR values to TIM3->CCR4
 * on every TIM3 update event so the entire frame — including the
 * mandatory >280 µs reset latch — is clocked out in hardware.
 *
 * Hardware requirement:
 *   The LED data line MUST be wired to PC9 (Daisy Seed2 DFM pin E8 / D3).
 *   That is the SDMMC1_D1 footprint pin on the Seed2 DFM, repurposed as
 *   TIM3_CH4 on this hardware revision.
 *
 * Usage:
 *   WS2812 leds;
 *   leds.Init(PIN_LED_DATA, 102);
 *   leds.SetPixel(0, 255, 0, 0);
 *   leds.Show();        // returns immediately, DMA does the rest
 */

#ifndef WS2812_H
#define WS2812_H

#include "daisy_seed.h"
#include <cstdint>

/**
 * Maximum number of LEDs supported by a single WS2812 instance.
 * Each LED uses 3 bytes of pixel storage plus 24 × 16-bit CCR values
 * (48 bytes) of DMA buffer.
 */
constexpr uint16_t WS2812_MAX_LEDS = 128;

class WS2812
{
  public:
    WS2812()  = default;
    ~WS2812() = default;

    /**
     * Initialise the driver.  Configures TIM3 channel 4 in PWM mode 1
     * with a period of 1.25 µs (800 kHz) and DMA1 Stream 7 in normal
     * memory-to-peripheral mode triggered by TIM3_UP.  PC9 is configured
     * as alternate-function (TIM3_CH4, AF2).
     *
     * @param pin       Ignored — retained for API compatibility.  PC9 is
     *                  always used as the LED data output.
     * @param num_leds  Number of LEDs in the chain (max WS2812_MAX_LEDS).
     */
    void Init(daisy::Pin pin, uint16_t num_leds);

    /** Set a single pixel colour (RGB order; stored internally as GRB). */
    void SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

    /** Turn all pixels off (zero the pixel buffer). */
    void Clear();

    /**
     * Transmit the pixel buffer to the LED strip via TIM3 + DMA.
     *
     * If a previous transfer is still in progress, Show() blocks until
     * it completes (typically < 4 ms for 102 LEDs incl. reset latch)
     * before starting the new one.  Use Busy() to poll without blocking.
     *
     * After the DMA transfer starts, the function returns immediately.
     * The timer and DMA engine handle the rest in hardware.
     */
    void Show();

    /**
     * Check whether a DMA transfer is currently in progress.
     * @return true while Show()'s transfer has not yet completed.
     */
    bool Busy() const;

    /** Configured number of LEDs. */
    uint16_t NumLeds() const { return num_leds_; }

  private:
    uint16_t num_leds_ = 0;

    /** RGB pixel buffer (caller-visible colour order). */
    uint8_t pixel_buf_[WS2812_MAX_LEDS * 3] = {};
};

#endif /* WS2812_H */
