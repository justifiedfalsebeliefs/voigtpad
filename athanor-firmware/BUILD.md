# Building and Deploying the Athanor Daisy Firmware

## Prerequisites

### 1. Install the ARM GCC Toolchain

**macOS** (Homebrew):
```bash
brew install --cask gcc-arm-embedded
```

### 2. Install dfu-util (for flashing)

**macOS**:
```bash
brew install dfu-util
```

---

## Clone and Initialize

```bash
git clone --recursive https://github.com/justifiedfalsebeliefs/athanor-firmware.git
cd athanor-firmware
```

If you already cloned without `--recursive`, initialize the submodules:

```bash
git submodule update --init --recursive
```

This pulls in **libDaisy** and its own dependencies (STM32 HAL drivers, CMSIS, USB
middleware).

---

## Build

### Step 1 — Build libDaisy (first time only, or after updating the submodule)

```bash
cd lib/libDaisy
make clean
make -j$(nproc)
cd ../..
```

You should see `build/libdaisy.a` created inside `lib/libDaisy/build/`.

### Step 2 — Build the firmware

```bash
make clean
make
```

Output files appear in `build/`:
| File | Purpose |
|------|---------|
| `athanor-daisy.bin` | Binary image for DFU flashing |
| `athanor-daisy.elf` | ELF with debug symbols (for GDB / OpenOCD) |
| `athanor-daisy.hex` | Intel HEX (alternative flash format) |
| `athanor-daisy.map` | Linker map (memory usage details) |

The build uses **`APP_TYPE = BOOT_SRAM`** by default. This means:
- The firmware is designed to be loaded by the **Daisy Bootloader**.
- It is flashed to QSPI flash at `0x90040000`.
- At boot, the bootloader copies it into SRAM (`0x24000000`) and executes it there.
- This supports firmware images up to ~450 KB.

---

## Flash / Deploy

### Step 1 — Enter the Daisy Bootloader

1. Connect the Daisy Seed2 DFM to your computer via USB.
2. **Hold** the **BOOT** button on the Seed.
3. While holding BOOT, **press and release** the **RESET** button.
4. Release the BOOT button.

The Seed is now in DFU bootloader mode. You can verify with:

```bash
dfu-util -l
```

You should see a device with `[0483:df11]`.

### Step 2 — Flash the firmware

```bash
make program-dfu
```

This runs:
```bash
dfu-util -a 0 -s 0x90040000:leave -D build/athanor-daisy.bin -d ,0483:df11
```

The `:leave` flag tells the bootloader to immediately boot into your firmware
after the transfer completes.

### Step 3 — Verify

After flashing:
- The onboard LED on the Seed2 DFM should turn **on** (solid) — this confirms
  the firmware is running.
- Connect a stereo audio source to the codec inputs (D1/D2) and a speaker or
  audio interface to the codec outputs. Audio should pass through unchanged
  (stereo passthrough).

---

## Quick Reference

```bash
# One-time: build libDaisy
cd lib/libDaisy && make -j$(nproc) && cd ../..

# Build firmware
make

# Flash (device must be in bootloader mode)
make program-dfu

# Clean everything
make clean
```

---

## What This Firmware Does

This is the **hello-world** foundation for the Athanor Daisy migration. It:

1. Initializes the Daisy Seed2 DFM hardware (clock, SDRAM, QSPI, PCM3060 codec).
2. Configures the SAI audio interface for **48 kHz / 24-bit stereo**.
3. Sets the audio block size to **24 samples** (matching the original engine's
   `ENGINE_BLOCK_SAMPLES`).
4. Runs a **stereo audio passthrough** — codec input is routed directly to
   codec output with no processing.
5. Turns on the onboard LED to signal that the firmware is alive.

### Next Steps

After validating basic audio passthrough, the following hardware will be
initialized and tested step by step:

- [ ] I2C bus D — PCAL6416AHF GPIO expanders (encoders, buttons)
- [ ] Encoder reading via GPIO expander interrupts
- [ ] Button reading (A, B, C, D, X)
- [ ] WS2812C LED strings (screen + encoder LEDs)
- [ ] I2C bus 1 — PCA9557PW gate outputs
- [ ] I2C bus 1 — MCP4728 DAC (channels 5–8)
- [ ] Internal STM32 DACs (channels 3–4)
- [ ] USB host on HS D+/D−
- [ ] Engine migration (control loop, processor chain, presets)

---

## Project Structure

```
athanor-firmware/
├── Makefile                 # Top-level build (APP_TYPE=BOOT_SRAM)
├── BUILD.md                 # ← You are here
├── src/
│   ├── main.cpp             # Entry point + audio passthrough
│   └── hardware.h           # Pin mapping & constants for all hardware
├── lib/
│   └── libDaisy/            # Electro-Smith libDaisy (git submodule)
└── build/                   # Build output (gitignored)
```
