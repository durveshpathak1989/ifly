/**
 * ============================================================
 *  FlySkyiBUS.h  —  FlySky FS-iA6B iBUS Receiver Driver
 *  ESP32 / Adafruit HUZZAH32 Feather (ESP32-WROOM-32E)
 *  FreeRTOS compatible
 * ============================================================
 *
 *  Hardware Connection
 *  -------------------
 *  FS-iA6B "S.BUS/iBUS" port  →  ESP32 GPIO 16 (UART2 RX)
 *  FS-iA6B VCC                →  5V  (or 4.0~6.5V BEC)
 *  FS-iA6B GND                →  GND
 *
 *  iBUS Protocol Summary
 *  ---------------------
 *  • 115200 baud, 8N1, single-wire serial (data flows one way: RX → FC)
 *  • Each frame is 32 bytes:
 *      Byte 0   : 0x20  (frame length)
 *      Byte 1   : 0x40  (command byte — sensor poll or channel data)
 *      Bytes 2–29: 14 channels × 2 bytes each (little-endian, 1000–2000 µs)
 *      Bytes 30–31: Checksum = 0xFFFF − sum(bytes 0..29), little-endian
 *  • Frame rate: ~7 ms (≈ 142 Hz)
 *
 *  Channel Mapping (FS-i6X default Mode 2)
 *  ----------------------------------------
 *    CH1  →  Roll       (Aileron)    Right stick L/R
 *    CH2  →  Pitch      (Elevator)   Right stick U/D
 *    CH3  →  Throttle                Left stick U/D
 *    CH4  →  Yaw        (Rudder)     Left stick L/R
 *    CH5  →  Flight Mode / Arm       SWA (2-pos) or SWB
 *    CH6  →  Aux / Tune              VrA knob
 *    CH7–10: Additional switches     SWB, SWC, SWD, VrB
 *
 *  Usage
 *  -----
 *    FlySkyiBUS rc;
 *    rc.begin(16, 17, 2);          // RX pin, TX pin (unused), UART num
 *    // In FreeRTOS task:
 *    rc.update();                  // call as often as possible
 *    int throttle = rc.getChannel(RC_CH_THROTTLE);  // 1000–2000 µs
 *    bool armed   = rc.isArmed();
 *    RCCommand cmd = rc.getCommand();
 *
 * ============================================================
 */

#ifndef FLYSKY_IBUS_H
#define FLYSKY_IBUS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ─────────────────────────────────────────────────────────────
//  Constants
// ─────────────────────────────────────────────────────────────
#define IBUS_FRAME_LEN      32
#define IBUS_CHANNELS       10        // FS-i6X supports up to 10
#define IBUS_START_BYTE1    0x20
#define IBUS_START_BYTE2    0x40
#define IBUS_BAUD           115200

// Channel index aliases (0-based)
// CONFIRMED WORKING LAYOUT (verified via [RAW] serial output):
//   CH1 Roll  CH2 Pitch  CH3 Throttle  CH4 Yaw
//   CH5 VrA   CH6 VrB    CH7 SWA       CH8 SWB   CH9 SWC   CH10 SWD
#define RC_CH_ROLL          0   // CH1   right stick L/R
#define RC_CH_PITCH         1   // CH2   right stick U/D
#define RC_CH_THROTTLE      2   // CH3   left  stick U/D
#define RC_CH_YAW           3   // CH4   left  stick L/R
#define RC_CH_FLIGHTMODE    6   // CH7   SWA   -- ARM/DISARM only
#define RC_CH_AUX1          4   // CH5   VrA   -- spare/PID tune
#define RC_CH_AUX2          7   // CH8   SWB   -- ANGLE/ACRO mode
#define RC_CH_AUX3          8   // CH9   SWC   -- accel confirm
#define RC_CH_AUX4          9   // CH10  SWD   -- CALIBRATION trigger only
#define RC_CH_AUX5          5   // CH6   VrB   -- spare/PID tune
// Stick thresholds
#define RC_THROTTLE_MIN     1000    // µs — zero throttle
#define RC_THROTTLE_MAX     2000    // µs — full throttle
#define RC_DEADBAND         50      // µs — centre deadband ±30 µs
#define RC_ARM_THRESHOLD    1700    // µs — CH5 above this = ARMED
#define RC_CALIB_THRESHOLD  1700    // µs — CH6 VrA above this = calib request (knob)
#define RC_SWD_THRESHOLD    1700    // µs — CH9 SWD above this = SWD HIGH

// Signal-loss detection — if no valid frame for this long, failsafe fires
#define RC_FAILSAFE_MS      500     // ms

// ─────────────────────────────────────────────────────────────
//  Derived flight command (normalised)
// ─────────────────────────────────────────────────────────────
enum class FlightMode : uint8_t {
    DISARMED = 0,
    ANGLE,          // self-level (outer + inner PID)
    ACRO,           // rate-only (inner PID)
    FAILSAFE        // RC signal lost
};

/**
 * RCCommand — normalised pilot intent, produced every update cycle.
 * All values ±1.0 (except throttle: 0.0–1.0).
 */
struct RCCommand {
    float roll;         // -1.0 (full left) … +1.0 (full right)
    float pitch;        // -1.0 (nose down) … +1.0 (nose up)
    float throttle;     //  0.0 (zero)      … +1.0 (full)
    float yaw;          // -1.0 (left)      … +1.0 (right)
    FlightMode mode;    // DISARMED / ANGLE / ACRO / FAILSAFE
    bool  swdHigh;      // true when SWD (CH10) is UP -- sole calib trigger
    bool  valid;        // false if signal lost
    uint16_t raw[IBUS_CHANNELS]; // raw µs for each channel
};

// ─────────────────────────────────────────────────────────────
//  FlySkyiBUS class
// ─────────────────────────────────────────────────────────────
class FlySkyiBUS {
public:
    /**
     * Constructor
     * @param serial  HardwareSerial instance to use (Serial2 recommended)
     */
    explicit FlySkyiBUS(HardwareSerial& serial = Serial2);

    /**
     * begin() — initialise UART and internal state.
     * @param rxPin   GPIO for UART RX (default 16)
     * @param txPin   GPIO for UART TX (not used, but required by HardwareSerial)
     * @param uartNum UART peripheral number (0,1,2 — use 2 to avoid conflicts)
     */
    void begin(int rxPin = 16, int txPin = 17, uint8_t uartNum = 2);

    /**
     * update() — read bytes from UART and parse frames.
     * Call this as frequently as possible (dedicated FreeRTOS task recommended).
     * Thread-safe: protects shared data with a mutex.
     */
    void update();

    // ── Accessors ──────────────────────────────────────────

    /**
     * getChannel() — raw µs value for a channel (1000–2000).
     * Returns 0 if no valid frame has been received yet.
     * @param ch  Channel index 0–9 (use RC_CH_* constants)
     */
    uint16_t getChannel(uint8_t ch) const;

    /**
     * getCommand() — normalised RCCommand struct, fully processed.
     * This is the primary interface for the flight controller.
     */
    RCCommand getCommand() const;

    /**
     * isArmed() — true when CH5 (FLIGHTMODE) exceeds ARM_THRESHOLD.
     */
    bool isArmed() const;

    /**
     * isSignalValid() — true if a valid frame was received within RC_FAILSAFE_MS.
     */
    bool isSignalValid() const;

    /**
     * getFrameRate() — measured frame rate in Hz (smoothed over 16 frames).
     */
    float getFrameRate() const;

    /**
     * getFailsafeCount() — number of failsafe events since boot.
     */
    uint32_t getFailsafeCount() const;

    /**
     * getChecksumFailCount() — cumulative count of frames that arrived
     * but failed the iBUS checksum. Watch its rate of increase to tell
     * corruption/mis-sync (climbs fast) from a clean lower link rate (flat).
     */
    uint32_t getChecksumFailCount() const;

    /**
     * printChannels() — Serial.print all 10 channels in µs.
     * For debug use only (blocks serial).
     */
    void printChannels() const;

    /**
     * printCommand() — Serial.print normalised RCCommand fields.
     */
    void printCommand() const;

private:
    HardwareSerial& _serial;

    // Frame buffer
    uint8_t  _buf[IBUS_FRAME_LEN];
    uint8_t  _bufIdx;

    // Parsed channel data (protected by mutex)
    uint16_t _channels[IBUS_CHANNELS];
    bool     _signalValid;
    uint32_t _lastFrameMs;
    uint32_t _frameCount;
    uint32_t _failsafeCount;
    uint32_t _csumFailCount = 0;   // frames that arrived but failed checksum (diagnostic)
    float    _frameRateHz;

    // FreeRTOS mutex for thread safety
    SemaphoreHandle_t _mutex;

    // Private helpers
    bool     _parseFrame();
    bool     _validateChecksum(const uint8_t* frame) const;
    float    _mapChannel(uint16_t raw, uint16_t centre = 1500,
                         uint16_t deadband = RC_DEADBAND) const;
    float    _mapThrottle(uint16_t raw) const;
    void     _updateFrameRate();
};

// ─────────────────────────────────────────────────────────────
//  Convenience: extern singleton declared in .cpp
// ─────────────────────────────────────────────────────────────
extern FlySkyiBUS rcReceiver;

#endif // FLYSKY_IBUS_H
