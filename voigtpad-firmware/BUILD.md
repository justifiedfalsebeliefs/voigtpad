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

* **B1 (top button)** — cycle pages: Page 1 (purple) ↔ Page 2 (cyan).
* **B2 (mid button)** — *deferred apply*: while held, pots are tracked
  visually with a breathing animation but parameters are frozen at their
  last applied value. Releasing B2 commits whatever the pots are reading
  at that instant. Lets you scrub a pot to a target without sweeping the
  audio through every value on the way.
* **B3 (bottom button)** — reserved for future use.

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
| P1  | Shimmer drift rate    | 0.15    | 0.01  | 1.0  |
| P2  | Shimmer octave (0-4)  | 2       | 0     | 4    |
| P3  | Shimmer air           | 0.5     | 0     | 1    |
| P4  | Fog cutoff (Hz)       | 4450    | 200   | 8000 |
| P5  | Sub warmth            | 0.2     | 0     | 1    |
| P6  | (unused)              | 0.5     | 0     | 1    |

### Always-on static parameters (not user-exposed)
* Stereo spread = 1.0
* Stereo width  = 1.0
* Sub pulse rate = 0.37 Hz
* Sub pulse depth = 0.6
* Fog mod depth = 0.4
* Fog mod rate  = 0.06 Hz

## Notes on the Faust patch

The original `docs/gas_chords.dsp` uses a slider for `master` and a richer
chord taxonomy (8 chords). This firmware:

* Removes MIDI; the engine always drones — no audio inputs are processed.
* Restricts the chord menu to the five musical options requested in the
  spec (`min;maj;min7;sus2;sus4`).
* Replaces the Faust `master` slider with a hard-coded master gain
  calibrated so the output begins to soft-clip only when all three of
  the *Main / Shimmer / Sub* level pots are at ≥ 75 %.
