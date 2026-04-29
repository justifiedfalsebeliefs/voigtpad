# Complete Pin Allocation — Daisy Seed 2 DFM 

## System overview

Eurorack module. Vision: configurable platform for DSP.
- Jacks 1 and 2 (in) and 9 and 10 (out) are Daisy audio L/R I/O via the onboard codec using the Daisy reference circuits.
- Jacks 3 - 8 are CV inputs, scaled with a classic -10V REF signal from =-5V to 0-3.3v
- There are 102 1010RGBC-WS2812B LEDs on one data line. The LED abstraction is already written.
- There are 6 pots and 3 buttons.
---

## Remaining pins

| Pinout | Daisy Pin | STM32 | ADC Channel | Notes |
|--------|-----------|-------|-------------|-------|
| C1 | D16 / A1 | PA3 | ADC1 | Pot 5 |
| C2 | D19 / A4 | PA6 | ADC4 | Pot 6 |
| C3 | D20 / A5 | PC1 | ADC5 | Pot 4 |
| C4 | D18 / A3 | PA7 | ADC3 | Pot 3 |
| C5 | D17 / A2 | PB1 | ADC2 | Pot 1 |
| C6 | D21 / A6 | PC4 | ADC6 | Pot 2 |
| C7 | D15 / A0 | PC0 | ADC0 | CV J6 |
| C8 | D23 / A8 | PA4 | ADC8 | CV J4 |
| C9 | D22 / A7 | PA5 | ADC7 | CV J3 |
| C10 | D31 / A12 | PC2 | ADC12 | CV J5 |
| D8 | D32 / A13 | PC3 | ADC13 | CV J7 |
| E1 | D24 / A9 | PA1 | ADC9 | CV J8 |
