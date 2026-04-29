# Building and Deploying the Voigtpad Daisy Firmware

Voigtpad runs on the same Daisy Seed2 DFM hardware as the Athanor module;
its build/flash workflow is identical.

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

This firmware lives inside the parent `voigtpad` repo:

```bash
cd voigtpad/voigtpad-firmware
git submodule add https://github.com/electro-smith/libDaisy.git lib/libDaisy
git submodule update --init --recursive
```

(If you already added the submodule, just run `git submodule update --init --recursive`.)

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
| `voigtpad-daisy.bin` | Binary image for DFU flashing |
| `voigtpad-daisy.elf` | ELF with debug symbols (for GDB / OpenOCD) |
| `voigtpad-daisy.hex` | Intel HEX (alternative flash format) |
| `voigtpad-daisy.map` | Linker map (memory usage details) |

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
dfu-util -a 0 -s 0x90040000:leave -D build/voigtpad-daisy.bin -d ,0483:df11
```

The `:leave` flag tells the bootloader to immediately boot into your firmware
after the transfer completes.

### Step 3 — Verify

After flashing:
- The onboard LED on the Seed2 DFM should turn **on** (solid) — this confirms
  the firmware is running.
- A stereo drone should be audible at the codec outputs immediately at boot.

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

## Project Structure

```
voigtpad-firmware/
├── Makefile              # Top-level build (APP_TYPE=BOOT_SRAM)
├── BUILD.md              # ← You are here
├── README.md             # Feature overview / pot mapping
├── src/
│   ├── main.cpp          # Entry point + audio callback wiring
│   ├── hardware.h        # Pin mapping (verbatim from athanor)
│   ├── ws2812.{h,cpp}    # WS2812 TIM3+DMA driver (verbatim)
│   ├── leds.{h,cpp}      # Panel LED layout helpers (verbatim)
│   ├── dsp.{h,cpp}       # gas_chords engine (Chord/Sub/Shimmer/Fog/Master)
│   └── controller.{h,cpp}# Pages, pot-catch, deferred apply, LED render
├── lib/
│   └── libDaisy/         # Electro-Smith libDaisy (git submodule)
└── build/                # Build output (gitignored)
```
