# Building and Deploying the Voigtpad Daisy Firmware

Voigtpad is an always-droning ambient texture generator inspired by Wolfgang
Voigt (GAS). It runs on the same Daisy Seed2 DFM hardware as the Athanor
module (6 pots, 3 buttons, 102 WS2812 LEDs); the synthesis engine is a
hand-translation of `docs/gas_chords.dsp` to C++.

## Prerequisites

ARM GCC Embedded toolchain (`arm-none-eabi-gcc`) and `dfu-util`. See
`athanor-firmware/BUILD.md` for platform-specific install commands.

## Initialise libDaisy

The repository's `voigtpad-firmware/lib/libDaisy` directory should be a
checkout of [libDaisy](https://github.com/electro-smith/libDaisy). If you
clone with `--recursive`, the submodule will be present; otherwise:

```bash
git submodule update --init --recursive
```

Build libDaisy once:

```bash
cd voigtpad-firmware/lib/libDaisy
make clean
make -j$(nproc)
cd ../..
```

## Build

```bash
cd voigtpad-firmware
make clean
make
```

Output `build/voigtpad-daisy.bin` is the firmware image.

## Flash (DFU)

1. Connect Daisy via USB.
2. Hold BOOT, tap RESET, release BOOT to enter the Daisy bootloader.
3. `make program-dfu`

## Hardware UX

* **B1 (top button)** — cycle pages: Page 1 ↔ Page 2.
* **B2 (mid button)** — *deferred apply*: while held, pots are tracked
  visually with a breathing animation but parameters are frozen at their
  last applied value. Releasing B2 commits whatever the pots are reading
  at that instant. Lets you scrub a pot to a target without sweeping the
  audio through every value on the way.
* **B3 (bottom button)** — reserved for future use.

### Pot animations

Each pot has its own animation chosen for the parameter it controls,
so the panel reads out the engine's internal state at a glance.

| Pot   | Param          | Animation                                                   |
|-------|----------------|-------------------------------------------------------------|
| P1P1  | Root pitch     | Filled arc, violet                                           |
| P1P2  | Shimmer level  | V/U meter, driven by the live shimmer-band peak (green→red) |
| P1P3  | Detune         | Filled arc, amber                                            |
| P1P4  | Main level     | V/U meter, driven by the live chord-band peak (green→red)   |
| P1P5  | Chord          | Discrete bands — each chord type has a mood colour, selected band lit |
| P1P6  | Sub level      | V/U meter, driven by the live sub-band peak (green→red)     |
| P2P1  | Shimmer drift  | Single pip swinging in lockstep with the live drift LFO value (CW rising / CCW falling), dim background |
| P2P2  | Shimmer octave | Vertical thermometer, red→blue ramp, lit up to selected band |
| P2P3  | Shimmer air    | Sparse white sparkles; spawn rate scales with the parameter |
| P2P4  | Fog cutoff     | Filled arc, sky-blue, driven by the *post-modulation* cutoff so the ring sweeps with the audible filter |
| P2P5  | Sub warmth     | Filled arc; colour mixes from off-white to deep red as warmth rises |
| P2P6  | Fog mod depth  | Filled arc, sky-blue                                         |

Uncaught pots still display the white pip at the *stored* (engine)
value so pot-catch behaviour is identical to V1.

## Pot mapping

### Page 1 (purple)
| Pot | Parameter           | Default | Min  | Max  |
|-----|---------------------|---------|------|------|
| P1  | Root pitch (MIDI)   | 36      | 24   | 60   |
| P2  | Shimmer level       | 0.5     | 0    | 1    |
| P3  | Detune (cents)      | 15      | 0    | 30   |
| P4  | Main level          | 0.5     | 0    | 1    |
| P5  | Chord (5 options)   | min     | min  | sus4 |
| P6  | Sub osc level       | 0.5     | 0    | 1    |

Chord options (in order): `min`, `maj`, `min7`, `sus2`, `sus4`.

### Page 2 (cyan)
| Pot | Parameter             | Default | Min   | Max  |
|-----|-----------------------|---------|-------|------|
| P1  | Shimmer drift rate    | 0.15    | 0.01  | 2.0  |
| P2  | Shimmer octave (0-4)  | 2       | 0     | 4    |
| P3  | Shimmer air           | 0.5     | 0     | 1    |
| P4  | Fog cutoff centre (Hz)| 4450    | 200   | 8000 |
| P5  | Sub warmth            | 0.2     | 0     | 1    |
| P6  | Fog mod depth         | 0.4     | 0     | 1    |

The fog cutoff is modulated by the shimmer-drift LFO; P6 sets how
much. At full depth the cutoff sweeps ±180% of the P4 centre value
(clamped to the engine's safe band) at the rate set by P1.

### Always-on static parameters (not user-exposed)
* Stereo spread = 1.0
* Stereo width  = 1.0
* Sub pulse rate = 0.37 Hz
* Sub pulse depth = 0.6

## Notes on the Faust patch

The original `docs/gas_chords.dsp` uses a slider for `master` and a richer
chord taxonomy (8 chords). This firmware:

* Removes MIDI; the engine always drones — no audio inputs are processed.
* Restricts the chord menu to the five musical options requested in the
  spec (`min;maj;min7;sus2;sus4`).
* Replaces the Faust `master` slider with a hard-coded master gain
  calibrated so the output begins to soft-clip only when all three of
  the *Main / Shimmer / Sub* level pots are at ≥ 75 %.
