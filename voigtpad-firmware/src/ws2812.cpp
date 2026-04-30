/**
 * @file ws2812.cpp
 * @brief WS2812B/C LED strip driver — TIM3_CH4 PWM + DMA1 Stream 7.
 *
 * Each WS2812 bit is encoded as one PWM cycle on TIM3 channel 4:
 *
 *      ┌──────┐                    ┌──────┐
 *  "0" │      │_______________     │ T0H  │___T0L___
 *      ▔                    ▔
 *      ┌─────────┐                 ┌─────────┐
 *  "1" │         │__________       │   T1H   │_T1L_
 *      ▔                  ▔
 *
 * For a 1.25 µs bit period (800 kHz):
 *   T0H ≈ 0.40 µs   (CCR ≈ 0.32 × ARR)
 *   T1H ≈ 0.80 µs   (CCR ≈ 0.64 × ARR)
 *
 * Both fall well inside the WS2812B datasheet windows (T0H 0.40 ± 0.15 µs,
 * T1H 0.80 ± 0.15 µs).  Period count and CCR values are computed at
 * runtime from the actual TIM3 kernel clock so the driver works
 * regardless of the system clock configuration.
 *
 * DMA streams CCR values from a half-word buffer to TIM3->CCR4 on every
 * TIM3 update event.  The buffer ends with kResetCells zero-valued cells
 * which hold the data line LOW for >280 µs to latch the frame.
 *
 * Resource usage (none of which collides with libDaisy on this build):
 *   - TIM3 channel 4
 *   - DMA1 Stream 7 with DMA_REQUEST_TIM3_UP
 *   - GPIO PC9 (AF2 = TIM3_CH4)
 */

#include "ws2812.h"
#include "daisy_core.h"       /* DMA_BUFFER_MEM_SECTION */

#include "stm32h7xx_hal.h"
#include <cstring>

/* ── Bit timing constants ────────────────────────────────────────────────── */

/** Number of WS2812 bits per LED (3 colour bytes × 8 bits). */
static constexpr uint32_t kBitsPerLed = 24;

/** Target bit-period frequency (WS2812 datasheet). */
static constexpr uint32_t kBitFreqHz  = 800000u;

/** Target T0H / T1H high-time as a fraction of the bit period. */
static constexpr float kT0HFrac = 0.32f;   /* ≈ 0.40 µs */
static constexpr float kT1HFrac = 0.64f;   /* ≈ 0.80 µs */

/**
 * Reset / latch cells: each cell is one full bit period (1.25 µs) with
 * CCR = 0 → line low.  240 cells ≈ 300 µs, comfortably above the 280 µs
 * minimum required by WS2812B/C latch timing.
 */
static constexpr uint16_t kResetCells = 240;

/** Maximum DMA cell count. */
static constexpr uint32_t kMaxCells =
    static_cast<uint32_t>(WS2812_MAX_LEDS) * kBitsPerLed + kResetCells;

/* ── DMA buffer (non-cached, DMA-accessible SRAM) ───────────────────────── */
/*
 * 16-bit half-word buffer — TIM3 CCR registers are 16-bit.  Placing the
 * buffer in the .sram1_bss section (via DMA_BUFFER_MEM_SECTION) puts it
 * in the non-cached SRAM1/2 region that DMA1 can reach without cache
 * maintenance.
 */
static uint16_t DMA_BUFFER_MEM_SECTION ccr_buf_[kMaxCells];

/* ── HAL handles ─────────────────────────────────────────────────────────── */

static TIM_HandleTypeDef htim3_;
static DMA_HandleTypeDef hdma_tim3_up_;

/* ── Cached encoding parameters ──────────────────────────────────────────── */

static uint16_t ccr_zero_  = 0;   /* CCR value for a "0" bit                 */
static uint16_t ccr_one_   = 0;   /* CCR value for a "1" bit                 */
static uint16_t xfer_len_  = 0;   /* Total cells per transfer (bits + reset) */

/* ── Transfer-in-progress flag ───────────────────────────────────────────── */
/*
 * Set true by Show() when DMA is started; cleared by the DMA TC callback.
 * The flag is the source of truth for Busy(); the HAL DMA state alone
 * isn't reliable across all HAL versions for "completed but not yet
 * stopped" transitions.
 */
static volatile bool xfer_busy_ = false;

/* ── Forward declarations ────────────────────────────────────────────────── */

static void     InitGpio();
static void     InitTim(uint32_t period_ticks);
static void     InitDma();
static uint32_t TimerKernelHz();
static void     EncodePixels(const uint8_t* pixels, uint16_t num_leds);
static void     DmaTcCb (DMA_HandleTypeDef* hdma);
static void     DmaErrCb(DMA_HandleTypeDef* hdma);

/* ========================================================================= */
/*  Public API                                                                */
/* ========================================================================= */

void WS2812::Init(daisy::Pin /* pin */, uint16_t num_leds)
{
    /*
     * NOTE: PC9 (TIM3_CH4) is always used as the LED data output.
     * The pin parameter is retained for API compatibility.
     */
    if (num_leds > WS2812_MAX_LEDS)
        num_leds = WS2812_MAX_LEDS;

    num_leds_ = num_leds;
    xfer_len_ = static_cast<uint16_t>(num_leds_ * kBitsPerLed + kResetCells);

    std::memset(pixel_buf_, 0, sizeof(pixel_buf_));
    std::memset(ccr_buf_,   0, sizeof(ccr_buf_));

    /* ── Compute period and CCR thresholds from the actual TIM clock ── */

    const uint32_t tim_hz       = TimerKernelHz();
    const uint32_t period_ticks = (tim_hz + (kBitFreqHz / 2)) / kBitFreqHz;
    /* period_ticks ≈ 250 at 200 MHz timer clock                       */

    ccr_zero_ = static_cast<uint16_t>(static_cast<float>(period_ticks) * kT0HFrac + 0.5f);
    ccr_one_  = static_cast<uint16_t>(static_cast<float>(period_ticks) * kT1HFrac + 0.5f);

    /* ── Hardware bring-up ───────────────────────────────────────────── */

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    InitGpio();
    InitDma();
    InitTim(period_ticks);

    /*
     * Enable the timer and CC4 output.  CCR4 is 0, so PC9 idles low.
     * The timer keeps free-running at 800 kHz forever — only the DMA
     * stream is gated per Show() to feed bit values.
     */
    __HAL_TIM_ENABLE(&htim3_);
}

void WS2812::SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= num_leds_)
        return;
    const uint16_t off = index * 3;
    pixel_buf_[off + 0] = g;   /* WS2812 native order is GRB */
    pixel_buf_[off + 1] = r;
    pixel_buf_[off + 2] = b;
}

void WS2812::Clear()
{
    std::memset(pixel_buf_, 0, sizeof(pixel_buf_));
}

void WS2812::Show()
{
    /* Wait for any previous transfer to finish. */
    while (xfer_busy_)
        ;

    /* Encode the current pixel buffer into TIM3 CCR values. */
    EncodePixels(pixel_buf_, num_leds_);

    /* Make sure the DMA stream is fully disabled before re-arming.   */
    __HAL_DMA_DISABLE(&hdma_tim3_up_);

    /*
     * Re-arm DMA1 Stream 7 from ccr_buf_ → TIM3->CCR4.
     * HAL_DMA_Start_IT clears flags, programs NDTR/M0AR/PAR, sets the
     * direction-appropriate IRQ flags, and enables the stream.
     */
    xfer_busy_ = true;
    HAL_DMA_Start_IT(&hdma_tim3_up_,
                     reinterpret_cast<uint32_t>(ccr_buf_),
                     reinterpret_cast<uint32_t>(&TIM3->CCR4),
                     xfer_len_);

    /*
     * Enable the TIM3 update DMA request.  From here, every TIM3
     * update event (every 1.25 µs) triggers DMA to load the next CCR
     * value into TIM3->CCR4.  After xfer_len_ events the DMA stream
     * auto-disables (normal mode) and DmaTcCb() clears xfer_busy_.
     */
    __HAL_TIM_ENABLE_DMA(&htim3_, TIM_DMA_UPDATE);
}

bool WS2812::Busy() const
{
    return xfer_busy_;
}

/* ========================================================================= */
/*  GPIO — PC9 (TIM3_CH4, AF2)                                                */
/* ========================================================================= */

static void InitGpio()
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_9;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;          /* Idle-low while CCR=0 */
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &gpio);
}

/* ========================================================================= */
/*  TIM3 — PWM mode 1 on CH4, period = 1.25 µs                                */
/* ========================================================================= */

static void InitTim(uint32_t period_ticks)
{
    std::memset(&htim3_, 0, sizeof(htim3_));

    htim3_.Instance               = TIM3;
    htim3_.Init.Prescaler         = 0;
    htim3_.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3_.Init.Period            = period_ticks - 1;
    htim3_.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3_.Init.RepetitionCounter = 0;
    htim3_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /*
     * Bypass any HAL_TIM_PWM_MspInit weak symbol — we own the GPIO/DMA
     * setup ourselves.  Setting State to BUSY makes HAL_TIM_*_Init skip
     * MspInit calls.
     */
    htim3_.State = HAL_TIM_STATE_BUSY;

    HAL_TIM_PWM_Init(&htim3_);

    TIM_OC_InitTypeDef oc = {};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 0;                     /* Idle low until DMA loads CCR */
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode   = TIM_OCFAST_DISABLE;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim3_, &oc, TIM_CHANNEL_4);

    /* Enable preload on CCR4 so a new compare value latches at update. */
    TIM3->CCMR2 |= TIM_CCMR2_OC4PE;

    /* Enable CC4 output — PC9 is now driven by the PWM compare engine. */
    TIM_CCxChannelCmd(TIM3, TIM_CHANNEL_4, TIM_CCx_ENABLE);
}

/* ========================================================================= */
/*  DMA1 Stream 7 — TIM3 update → TIM3->CCR4                                  */
/* ========================================================================= */

static void InitDma()
{
    std::memset(&hdma_tim3_up_, 0, sizeof(hdma_tim3_up_));

    hdma_tim3_up_.Instance                 = DMA1_Stream7;
    hdma_tim3_up_.Init.Request             = DMA_REQUEST_TIM3_UP;
    hdma_tim3_up_.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim3_up_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim3_up_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim3_up_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim3_up_.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tim3_up_.Init.Mode                = DMA_NORMAL;
    hdma_tim3_up_.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_tim3_up_.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_tim3_up_);

    /* Direct callbacks — bypass any HAL_TIM weak callbacks libDaisy may
     * happen to define elsewhere.                                       */
    hdma_tim3_up_.XferCpltCallback     = DmaTcCb;
    hdma_tim3_up_.XferHalfCpltCallback = nullptr;
    hdma_tim3_up_.XferErrorCallback    = DmaErrCb;

    /*
     * Priority 5: well below audio DMA (priority 0).  LED jitter is
     * irrelevant; audio must never be preempted by LED transfers.
     */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

/* ========================================================================= */
/*  Pixel → CCR encoding                                                      */
/* ========================================================================= */

static void EncodePixels(const uint8_t* pixels, uint16_t num_leds)
{
    uint16_t*      dst         = ccr_buf_;
    const uint16_t total_bytes = static_cast<uint16_t>(num_leds * 3);
    const uint16_t one         = ccr_one_;
    const uint16_t zero        = ccr_zero_;

    for (uint16_t i = 0; i < total_bytes; i++)
    {
        uint8_t byte = pixels[i];
        /* MSB first per WS2812 wire format. */
        for (uint8_t b = 0; b < 8; b++)
        {
            *dst++ = (byte & 0x80) ? one : zero;
            byte <<= 1;
        }
    }

    /* Reset / latch period: zero CCR → line low for ≥ 280 µs. */
    for (uint16_t i = 0; i < kResetCells; i++)
        *dst++ = 0;
}

/* ========================================================================= */
/*  Timer kernel clock query                                                  */
/* ========================================================================= */
/*
 * On STM32H7, TIM3 lives on APB1.  Its kernel clock equals the APB1 timer
 * clock, which is:
 *
 *   - APB1 clock          if D2PPRE1 == /1   (no prescaling)
 *   - 2 × APB1 clock      if D2PPRE1 >= /2   (any prescaling)
 *
 * The TIMPRE bit in RCC->CFGR can override this with a 4× rule, but
 * libDaisy does not enable it.  Default Daisy clock tree gives 200 MHz.
 */
static uint32_t TimerKernelHz()
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t d2ppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;

    /* d2ppre1 < 4 → AHB / 1 (no prescaling); ≥ 4 → AHB / 2,4,8,16. */
    const uint32_t mult = (d2ppre1 < 4) ? 1u : 2u;
    uint32_t hz = pclk1 * mult;

    /* Fallback if clock query failed for any reason. */
    if (hz == 0)
        hz = 200000000u;

    return hz;
}

/* ========================================================================= */
/*  DMA callbacks (ISR context)                                               */
/* ========================================================================= */

static void DmaTcCb(DMA_HandleTypeDef* /* hdma */)
{
    /*
     * Stop further TIM3 update events from triggering DMA.  The DMA
     * stream auto-disabled when NDTR reached 0 (normal mode), so no
     * additional bus traffic happens — but disabling UDE is good
     * hygiene and avoids any lingering request line state.
     */
    __HAL_TIM_DISABLE_DMA(&htim3_, TIM_DMA_UPDATE);
    xfer_busy_ = false;
}

static void DmaErrCb(DMA_HandleTypeDef* /* hdma */)
{
    /* Treat errors as transfer-complete: clear the busy flag so the
     * next Show() can start a fresh transfer.  Visual artefacts are
     * acceptable here; the next frame will overwrite the strip.       */
    __HAL_TIM_DISABLE_DMA(&htim3_, TIM_DMA_UPDATE);
    xfer_busy_ = false;
}

/* ── DMA1 Stream 7 IRQ (not used by libDaisy — safe to define) ──────────── */

extern "C" void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim3_up_);
}
