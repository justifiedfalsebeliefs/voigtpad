# =============================================================================
# Voigtpad Daisy Firmware - Makefile
# Target: Daisy Seed2 DFM (STM32H750, PCM3060 codec)
# Build: BOOT_SRAM (Daisy Bootloader compatible, firmware in QSPI → runs from SRAM)
# =============================================================================

TARGET = voigtpad-daisy

# Sources
CPP_SOURCES = src/main.cpp src/ws2812.cpp src/leds.cpp src/dsp.cpp src/controller.cpp

# Library Locations
LIBDAISY_DIR = lib/libDaisy

# Use BOOT_SRAM for Daisy bootloader compatibility.
# Firmware is flashed to QSPI (0x90040000) and the bootloader copies it to
# SRAM (0x24000000) for execution. This supports large firmware images and
# preserves the bootloader in internal flash.
APP_TYPE = BOOT_SRAM

# Core location and generic Makefile
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
