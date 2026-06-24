# Test Quad iFly / FlySky iBUS Library

## Explain It Simply

This module listens to the radio receiver. It turns your transmitter sticks and switches into numbers the flight controller can understand.

This library reads FlySky FS-iA6B iBUS receiver channels over ESP32 UART2.

## Pin Map

| Receiver signal | ESP32 pin | Notes |
| --- | ---: | --- |
| iBUS TX | GPIO 16 | Receiver serial data into ESP32 UART2 RX |
| ESP32 TX | GPIO 4 | Optional/spare UART2 TX |
| VCC | 5V or receiver rail | Match receiver requirements |
| GND | GND | Common ground with ESP32 and ESCs |

## Main INO Integration Example

```cpp
#include "FlySkyiBUS.h"

#define PIN_IBUS_RX 16
#define PIN_IBUS_TX 4

void setup() {
    rcReceiver.begin(PIN_IBUS_RX, PIN_IBUS_TX, 2);
}

void loop() {
    rcReceiver.update();
    uint16_t throttle = rcReceiver.getChannel(RC_CH_THROTTLE);
    RCCommand cmd = rcReceiver.getCommand();
}
```


## Why These Data Types

RC pulse/channel values are integer microsecond-style values, which match receiver protocol output and make arming thresholds deterministic. The firmware converts them to `float` setpoints only after applying deadbands and scaling.
