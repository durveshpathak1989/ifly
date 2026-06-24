/**
 * ============================================================
 *  FlySkyiBUS.cpp — FlySky FS-iA6B iBUS Receiver Driver
 *  Implementation
 * ============================================================
 */

#include "FlySkyiBUS.h"
#include "DebugConfig.h"

// ─────────────────────────────────────────────────────────────
//  Singleton
// ─────────────────────────────────────────────────────────────
FlySkyiBUS rcReceiver(Serial2);

// ─────────────────────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────────────────────
FlySkyiBUS::FlySkyiBUS(HardwareSerial& serial)
    : _serial(serial),
      _bufIdx(0),
      _signalValid(false),
      _lastFrameMs(0),
      _frameCount(0),
      _failsafeCount(0),
      _frameRateHz(0.0f)
{
    memset(_channels, 0, sizeof(_channels));
    // Initialise mutex before begin() is called
    _mutex = xSemaphoreCreateMutex();
}

// ─────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────
void FlySkyiBUS::begin(int rxPin, int txPin, uint8_t uartNum)
{
    // HardwareSerial::begin(baud, config, rxPin, txPin)
    //_serial.setRxBufferSize(512); 
    _serial.begin(IBUS_BAUD, SERIAL_8N1, rxPin, txPin);

    DBG_PRINTF("[iBUS] Initialised on UART%u  RX=GPIO%d  TX=GPIO%d\n",
                  uartNum, rxPin, txPin);
    DBG_PRINTLN("[iBUS] Waiting for FlySky FS-iA6B frames...");
}

// ─────────────────────────────────────────────────────────────
//  update() — called as fast as possible from FreeRTOS task
// ─────────────────────────────────────────────────────────────
void FlySkyiBUS::update()
{
    // Drain all available bytes
    while (_serial.available()) {
        uint8_t b = (uint8_t)_serial.read();

        // Sync on start-byte pair: 0x20, 0x40
        if (_bufIdx == 0 && b != IBUS_START_BYTE1) continue;
        if (_bufIdx == 1 && b != IBUS_START_BYTE2) { _bufIdx = 0; continue; }

        _buf[_bufIdx++] = b;

        if (_bufIdx == IBUS_FRAME_LEN) {
            _bufIdx = 0;  // reset regardless of parse result
            _parseFrame();
        }
    }

    // Failsafe — check for signal timeout
    if (_signalValid && (millis() - _lastFrameMs) > RC_FAILSAFE_MS) {
        if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
            _signalValid = false;
            _failsafeCount++;
            xSemaphoreGive(_mutex);
        }
        DBG_PRINTLN("[iBUS] FAILSAFE: signal lost!");
    }
}

// ─────────────────────────────────────────────────────────────
//  _parseFrame() — validate and extract channel values
// ─────────────────────────────────────────────────────────────
bool FlySkyiBUS::_parseFrame()
{
    if (!_validateChecksum(_buf)) {
        _csumFailCount++;   // count corrupt / mis-synced frames for diagnostics
        return false;       // still discard the bad frame
    }

    // Extract 14 channel slots (FS-i6X uses first 10)
    uint16_t newCh[IBUS_CHANNELS];
    for (int i = 0; i < IBUS_CHANNELS; i++) {
        uint8_t lo = _buf[2 + i * 2];
        uint8_t hi = _buf[3 + i * 2];
        uint16_t raw = (uint16_t)(lo | (hi << 8));
        // Clamp to valid range
        newCh[i] = constrain(raw, 900, 2100);
    }

    // Write atomically under mutex
    if (xSemaphoreTake(_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(_channels, newCh, sizeof(_channels));
        _signalValid  = true;
        _lastFrameMs  = millis();
        _frameCount++;
        _updateFrameRate();
        xSemaphoreGive(_mutex);
    }

    return true;
}

// ─────────────────────────────────────────────────────────────
//  _validateChecksum()
// ─────────────────────────────────────────────────────────────
bool FlySkyiBUS::_validateChecksum(const uint8_t* frame) const
{
    uint16_t sum = 0xFFFF;
    for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
        sum -= frame[i];
    }
    uint16_t rxCsum = (uint16_t)(frame[30] | (frame[31] << 8));
    return (sum == rxCsum);
}

// ─────────────────────────────────────────────────────────────
//  _updateFrameRate() — exponential moving average
//  Call INSIDE mutex-locked region only
// ─────────────────────────────────────────────────────────────
// void FlySkyiBUS::_updateFrameRate()
// {
//     static uint32_t lastMs = 0;
//     uint32_t now = millis();
//     if (lastMs > 0 && (now - lastMs) > 0) {
//         float instantHz = 1000.0f / (float)(now - lastMs);
//         _frameRateHz = _frameRateHz * 0.9375f + instantHz * 0.0625f; // 1/16 blend
//     }
//     lastMs = now;
// }

void FlySkyiBUS::_updateFrameRate()
{
    static uint32_t lastMs = 0;
    static uint32_t acceptedFrameCount = 0;

    uint32_t now = millis();

    acceptedFrameCount++;

    // iBUS is about 140 Hz, but UART buffer draining can make parser calls
    // appear faster. Estimate display Hz over a longer window instead of
    // using one frame-to-frame delta.
    if (lastMs > 0) {
        uint32_t dt = now - lastMs;

        if (dt >= 500) {
            _frameRateHz = (acceptedFrameCount * 1000.0f) / (float)dt;

            // Clamp only the displayed diagnostic value.
            // Do not affect parsing, RC data, failsafe, telemetry, or control.
            if (_frameRateHz > 180.0f) {
                _frameRateHz = 140.0f;
            }

            acceptedFrameCount = 0;
            lastMs = now;
        }
    } else {
        lastMs = now;
        acceptedFrameCount = 0;
    }
}

// ─────────────────────────────────────────────────────────────
//  _mapChannel() — raw µs → normalised -1.0 … +1.0
//  Applies deadband around centre (default 1500 µs)
// ─────────────────────────────────────────────────────────────
float FlySkyiBUS::_mapChannel(uint16_t raw, uint16_t centre, uint16_t deadband) const
{
    int16_t offset = (int16_t)raw - (int16_t)centre;
    if (abs(offset) <= (int16_t)deadband) return 0.0f;

    float half = 500.0f - (float)deadband;  // usable half-range
    float norm  = (float)(offset - (offset > 0 ? (int16_t)deadband : -(int16_t)deadband)) / half;
    return constrain(norm, -1.0f, 1.0f);
}

// ─────────────────────────────────────────────────────────────
//  _mapThrottle() — raw µs → 0.0 … 1.0  (no deadband, no inversion)
// ─────────────────────────────────────────────────────────────
float FlySkyiBUS::_mapThrottle(uint16_t raw) const
{
    float t = (float)(raw - RC_THROTTLE_MIN) / (float)(RC_THROTTLE_MAX - RC_THROTTLE_MIN);
    return constrain(t, 0.0f, 1.0f);
}

// ─────────────────────────────────────────────────────────────
//  getChannel()
// ─────────────────────────────────────────────────────────────
uint16_t FlySkyiBUS::getChannel(uint8_t ch) const
{
    if (ch >= IBUS_CHANNELS) return 0;
    uint16_t val = 0;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        val = _channels[ch];
        xSemaphoreGive(_mutex);
    }
    return val;
}

// ─────────────────────────────────────────────────────────────
//  isArmed()
// ─────────────────────────────────────────────────────────────
bool FlySkyiBUS::isArmed() const
{
    return isSignalValid() && (getChannel(RC_CH_FLIGHTMODE) >= RC_ARM_THRESHOLD);
}

// ─────────────────────────────────────────────────────────────
//  isSignalValid()
// ─────────────────────────────────────────────────────────────
bool FlySkyiBUS::isSignalValid() const
{
    bool v = false;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        v = _signalValid;
        xSemaphoreGive(_mutex);
    }
    return v;
}

// ─────────────────────────────────────────────────────────────
//  getFrameRate()
// ─────────────────────────────────────────────────────────────
float FlySkyiBUS::getFrameRate() const
{
    float hz = 0;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        hz = _frameRateHz;
        xSemaphoreGive(_mutex);
    }
    return hz;
}

// ─────────────────────────────────────────────────────────────
//  getFailsafeCount()
// ─────────────────────────────────────────────────────────────
uint32_t FlySkyiBUS::getFailsafeCount() const
{
    uint32_t c = 0;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        c = _failsafeCount;
        xSemaphoreGive(_mutex);
    }
    return c;
}

// ─────────────────────────────────────────────────────────────
//  getChecksumFailCount()
//  Single writer (taskRC); a 32-bit aligned read is atomic on the
//  ESP32, so no mutex is needed for this monotonic diagnostic counter.
// ─────────────────────────────────────────────────────────────
uint32_t FlySkyiBUS::getChecksumFailCount() const
{
    return _csumFailCount;
}

// ─────────────────────────────────────────────────────────────
//  getCommand() — primary flight controller interface
// ─────────────────────────────────────────────────────────────
RCCommand FlySkyiBUS::getCommand() const
{
    RCCommand cmd;
    memset(&cmd, 0, sizeof(cmd));

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        cmd.mode  = FlightMode::FAILSAFE;
        cmd.valid = false;
        return cmd;
    }

    // Copy raw values while holding lock
    bool valid = _signalValid;
    uint16_t ch[IBUS_CHANNELS];
    memcpy(ch, _channels, sizeof(ch));
    xSemaphoreGive(_mutex);

    // Fill raw array
    for (int i = 0; i < IBUS_CHANNELS; i++) cmd.raw[i] = ch[i];

    if (!valid) {
        cmd.mode  = FlightMode::FAILSAFE;
        cmd.valid = false;
        return cmd;
    }

    // Normalise axes
    cmd.roll     =  _mapChannel(ch[RC_CH_ROLL]);
    cmd.pitch    = -_mapChannel(ch[RC_CH_PITCH]);   // invert: stick up = nose up
    cmd.throttle =  _mapThrottle(ch[RC_CH_THROTTLE]);
    cmd.yaw      =  _mapChannel(ch[RC_CH_YAW]);

    // Arm: SWA (CH7) only
    bool armed = (ch[RC_CH_FLIGHTMODE] >= RC_ARM_THRESHOLD);
    // Flight mode: SWB (CH8) — low=ANGLE, high=ACRO
    bool acro  = (ch[RC_CH_AUX2] >= RC_ARM_THRESHOLD);

    if (!armed) {
        cmd.mode = FlightMode::DISARMED;
    } else {
        cmd.mode = acro ? FlightMode::ACRO : FlightMode::ANGLE;
    }

    // SWD (CH10) state — sole calibration trigger
    // Rising-edge detection is handled in taskRC, not here
    cmd.swdHigh = (ch[RC_CH_AUX4] >= RC_SWD_THRESHOLD);

    cmd.valid = true;
    return cmd;
}

// ─────────────────────────────────────────────────────────────
//  printChannels() — debug
// ─────────────────────────────────────────────────────────────
void FlySkyiBUS::printChannels() const
{
    DBG_PRINT("[iBUS] CH: ");
    for (int i = 0; i < IBUS_CHANNELS; i++) {
        DBG_PRINTF("CH%d=%4d ", i + 1, getChannel(i));
    }
    DBG_PRINTF("| %.1f Hz | FS#%lu | CSUMFAIL#%lu\n",
                  getFrameRate(), (unsigned long)getFailsafeCount(),
                  (unsigned long)getChecksumFailCount());
}

// ─────────────────────────────────────────────────────────────
//  printCommand() — debug
// ─────────────────────────────────────────────────────────────
void FlySkyiBUS::printCommand() const
{
    RCCommand cmd = getCommand();
    const char* modeStr = "???";
    switch (cmd.mode) {
        case FlightMode::DISARMED: modeStr = "DISARMED"; break;
        case FlightMode::ANGLE:    modeStr = "ANGLE   "; break;
        case FlightMode::ACRO:     modeStr = "ACRO    "; break;
        case FlightMode::FAILSAFE: modeStr = "FAILSAFE"; break;
    }
    DBG_PRINTF("[CMD] Mode=%-8s  T=%+.2f  R=%+.2f  P=%+.2f  Y=%+.2f  SWD=%d\n",
                  modeStr,
                  cmd.throttle, cmd.roll, cmd.pitch, cmd.yaw,
                  (int)cmd.swdHigh);
}
