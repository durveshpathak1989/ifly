# FlySky iBUS Receiver Library

## Purpose

Parses FlySky FS-iA6B iBUS serial frames, validates checksums, tracks failsafe state, maps channels into flight commands, and exposes RC diagnostics.

## Files

- `FlySkyiBUS.h/.cpp`: UART reader, frame parser, channel mapper, failsafe handling, command model.

## Quick Start

```cpp
#include "FlySkyiBUS.h"

void setup() {
    rcReceiver.begin(16, 4, 2);
}

void loop() {
    rcReceiver.update();
    RCCommand cmd = rcReceiver.getCommand();
    if (cmd.valid && cmd.mode == FlightMode::ANGLE) {
        float roll = cmd.roll;
    }
}
```

## How It Fits Into The Flight Controller

This library lives under `Submodules/iFly` in the main `Test_Quad` firmware
and is built as an Arduino library by adding `Submodules/` to the Arduino
library search path. The main firmware includes it directly from
`RC_FlightController.ino` or from another support module.

The flight controller runs a 400 Hz control loop on ESP32, so this library
should avoid heap allocation, long blocking calls, and unbounded Serial output
inside flight-critical paths. Debug output should use `DebugConfig.h` macros
where available so `VERBOSE_ON=0` builds can compile prints out.

## Data Type Choices

- `uint16_t` channels: RC pulse-style channel values are typically 1000-2000 us and fit in 16 bits.
- `uint8_t` frame bytes: iBUS is a byte protocol with a fixed 32-byte frame.
- `RCCommand`: Separates raw channels from normalized control intent for the flight controller.
- `SemaphoreHandle_t`: The RC task and control task share data across FreeRTOS tasks; mutex protection prevents torn reads.

## Usage Guidance

1. Initialize hardware-facing classes once during `setup()`.
2. Keep update/read calls deterministic when used from a FreeRTOS task.
3. Prefer explicit validity flags over sentinel numeric values.
4. Keep units visible in field names, such as `_dps`, `_g`, `_uT`, `_m`, or `_us`.
5. When adding telemetry fields, update both the packet struct and JSON serializer.

## Example Build Integration

```bash
arduino-cli compile \
  --fqbn esp32:esp32:esp32:UploadSpeed=921600,CPUFreq=240,FlashFreq=80,FlashMode=qio,FlashSize=4M,PartitionScheme=min_spiffs,DebugLevel=none,PSRAM=disabled,LoopCore=1,EventsCore=1,EraseFlash=none,JTAGAdapter=default,ZigbeeMode=default \
  --libraries ./Submodules \
  .
```

For quiet flight builds:

```bash
arduino-cli compile ... --build-property compiler.cpp.extra_flags=-DVERBOSE_ON=0
```


## Integration Notes

In the main flight-controller sketch, this library is included through Arduino's
library search path. When this folder is converted to a git submodule, keep the
folder name stable under `Submodules/` so includes such as `#include "..."`
continue to resolve.

Most examples below are intentionally small. On the real flight controller,
objects are usually constructed globally, initialized once from `setup()`, and
then called from FreeRTOS tasks at deterministic rates.

