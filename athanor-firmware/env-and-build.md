# Environment Setup, Build, and Deploy

Complete instructions for setting up your development environment, building the
Athanor Daisy firmware, and flashing it to hardware.

---

## 1. Environment Setup (One-Time)

### 1.1 Install the ARM GCC Toolchain

The firmware targets an STM32H750 (Cortex-M7). You need the ARM embedded GCC
toolchain to cross-compile.

**macOS (Homebrew):**

```bash
brew install --cask gcc-arm-embedded
```

Verify the installation:

```bash
arm-none-eabi-gcc --version
```

### 1.2 Install dfu-util

`dfu-util` is used to flash firmware over USB DFU.

**macOS (Homebrew):**

```bash
brew install dfu-util
```

Verify:

```bash
dfu-util --version
```

### 1.3 Clone the Repository (with Submodules)

The project depends on **libDaisy** as a git submodule. You must clone
recursively to pull it in:

```bash
git clone --recursive https://github.com/justifiedfalsebeliefs/athanor-firmware.git
cd athanor-firmware
```

If you already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

### 1.4 Build libDaisy

libDaisy must be compiled once before you can build the firmware. You also need
to rebuild it any time you update the libDaisy submodule.

```bash
cd lib/libDaisy
make clean
make -j$(sysctl -n hw.logicalcpu)
cd ../..
```

> **Note:** On Linux, replace `$(sysctl -n hw.logicalcpu)` with `$(nproc)`.

Confirm that `lib/libDaisy/build/libdaisy.a` exists after the build completes.

### 1.5 Flash the Daisy Bootloader (Once Per Daisy Seed)

This project uses `APP_TYPE = BOOT_SRAM`. The firmware is stored in external
QSPI flash (`0x90040000`) and the **Daisy bootloader** copies it into SRAM for
execution at boot. This means the Daisy bootloader must be installed in the
Seed's internal flash before you can deploy application firmware.

**You only need to do this once per physical Daisy Seed** (unless you erase
internal flash).

#### Put the Seed into stock STM32 DFU mode

1. Connect the Daisy Seed2 DFM to your computer via USB.
2. **Hold** the **BOOT** button on the Seed.
3. While holding BOOT, **press and release** the **RESET** button.
4. Release the BOOT button.

#### Verify the device is in DFU mode

```bash
dfu-util -l
```

You should see a device with `[0483:df11]` and `@Internal Flash` in the
description. If you see `@Flash /0x90000000/...` instead, the Daisy bootloader
is already installed — skip to Section 3.

#### Flash the bootloader

```bash
make program-boot
```

This runs:

```
dfu-util -a 0 -s 0x08000000:leave -D lib/libDaisy/core/dsy_bootloader_v6_4-intdfu-2000ms.bin -d ,0483:df11
```

The bootloader binary (`dsy_bootloader_v6_4-intdfu-2000ms.bin`) is included in
the libDaisy submodule. The `intdfu-2000ms` variant means the bootloader waits
2 seconds in DFU mode on every boot before launching the application — this
gives you a window to reflash without needing to hold buttons.

> **Ignore the trailing error** `dfu-util: Error during download get_status` —
> this is normal and occurs because the device resets before the USB status
> response completes.

After flashing, the Seed will reboot into the Daisy bootloader. Verify by
running `dfu-util -l` again — you should now see:

```
@Flash /0x90000000/64*4Kg/0x90040000/60*64Kg/0x90400000/60*64Kg
```

This confirms QSPI flash is exposed and you're ready to deploy firmware.

---

## 2. Build

From the project root:

```bash
make clean   # optional — only needed if you want a full rebuild
make
```

Build output appears in `build/`:

| File                  | Purpose                                       |
| --------------------- | --------------------------------------------- |
| `athanor-daisy.bin`   | Binary image for DFU flashing                 |
| `athanor-daisy.elf`   | ELF with debug symbols (for GDB / OpenOCD)    |
| `athanor-daisy.hex`   | Intel HEX (alternative flash format)          |
| `athanor-daisy.map`   | Linker map (memory usage details)             |

---

## 3. Deploy (Flash to Hardware)

### 3.1 Enter DFU Mode

The Daisy bootloader enters DFU mode for 2 seconds on every reset. You have two
options:

**Option A — Press RESET and flash quickly:**

The bootloader waits 2 seconds in DFU mode after every reset. Press RESET, then
immediately run the flash command. This is the fastest workflow once you're used
to the timing.

**Option B — Hold BOOT + press RESET (guaranteed):**

1. **Hold** the **BOOT** button.
2. While holding BOOT, **press and release RESET**.
3. Release BOOT.

This puts the device into DFU mode indefinitely (no 2-second timeout).

### 3.2 Flash the Firmware

```bash
make program-dfu
```

This runs:

```
dfu-util -a 0 -s 0x90040000:leave -D build/athanor-daisy.bin -d ,0483:df11
```

The `:leave` flag tells the bootloader to immediately boot into your firmware
after the transfer completes.

### 3.3 Verify

After flashing, the firmware should start automatically. Currently (hello-world
stage), the onboard LED turns on solid to confirm the firmware is running.

---

## Troubleshooting

### `dfu-util: Last page at 0x90051e3f is not writeable`

The Daisy bootloader is **not installed**. The device is in stock STM32 DFU mode
and only exposes internal flash. QSPI flash (`0x90040000`) is not available.

**Fix:** Flash the Daisy bootloader first — see Section 1.5.

### `dfu-util: No DFU capable USB device available`

The device is not in DFU mode. Hold BOOT, press RESET, release BOOT, and try
again.

### `make program-dfu` exits immediately with no device found

Make sure you're within the bootloader's 2-second DFU window, or use the
BOOT+RESET method to stay in DFU indefinitely.

### libDaisy build errors after `git pull`

If the libDaisy submodule was updated, rebuild it:

```bash
git submodule update --init --recursive
cd lib/libDaisy && make clean && make -j$(sysctl -n hw.logicalcpu) && cd ../..
```

---

## Quick Reference Summary

### Run Once Per Dev Machine

Install toolchain and flashing tools:

```bash
brew install --cask gcc-arm-embedded
brew install dfu-util
```

### Run Once Per Clone / Submodule Update

Build the libDaisy library:

```bash
cd lib/libDaisy && make clean && make -j$(sysctl -n hw.logicalcpu) && cd ../..
```

### Run Once Per Daisy Seed

Flash the Daisy bootloader (device must be in stock STM32 DFU mode — hold BOOT,
press RESET, release BOOT):

```bash
make program-boot
```

### Run to Build

```bash
make
```

### Run to Deploy

Put device into DFU mode (press RESET, or hold BOOT + press RESET), then:

```bash
make program-dfu
```
