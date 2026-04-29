# Voigtpad Daisy Firmware

Generative ambient drone synth running on the Daisy Seed2 DFM hardware
platform.  This firmware is a port of the `gas_chords.dsp` Faust patch
(see `../docs/gas_chords.dsp`) onto the Athanor panel hardware
(6 pots × 3 buttons × 102 WS2812 LEDs).

## What it does

Always-on stereo drone — no input audio is processed.  Four sound layers
(per the Faust patch) are summed to a soft-saturated master bus:

1. **Chord** — 4-voice super-saw chord (root + chord intervals), each
   voice through a 2.2 kHz LP, panned across full stereo spread.
2. **Sub Drone** — 3-harmonic sub oscillator (root −12 semitones), with
   warmth-controlled `tanh` saturation and a fixed 0.37 Hz pulse.
3. **Shimmer** — 4 detuned bell partials transposed +N octaves, plus
   filtered pink-noise "air", with slow drift LFOs and full stereo width.
4. **Fog** — 3rd-order low-pass on the summed bus, with cutoff slowly
   modulated by a 0.06 Hz LFO (depth 0.4, both fixed).

## UI

* Pot-catch (verbatim from athanor) so values don't jump on page change.
* Mid button held → **deferred apply**: pots edit in display only,
  changes commit on release.  Useful for the discrete-band pots.
* Top button cycles between two pages.  Bottom button reserved.

### Page 1 (purple)
| Pot | Param          | Default | Range            |
|-----|----------------|---------|------------------|
| P1  | Root pitch     | MIDI 36 | MIDI 24..60      |
| P2  | Shimmer level  | 0.5     | 0..1             |
| P3  | Detune (cents) | 15      | 0..30            |
| P4  | Main level     | 0.5     | 0..1             |
| P5  | Chord (band)   | min     | min/maj/min7/sus2/sus4 |
| P6  | Sub level      | 0.5     | 0..1             |

### Page 2 (cyan)
| Pot | Param           | Default | Range          |
|-----|-----------------|---------|----------------|
| P1  | Shimmer drift   | 0.15 Hz | 0.01..1.0 Hz   |
| P2  | Shimmer octave  | 2       | 0..4 (5 bands) |
| P3  | Shimmer air     | 0.5     | 0..1           |
| P4  | Fog cutoff      | 4450 Hz | 200..8000 Hz   |
| P5  | Sub warmth      | 0.2     | 0..1           |
| P6  | (reserved)      | —       | —              |

See `BUILD.md` for build / flash instructions.
