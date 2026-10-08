
#pragma once

#include <Arduino.h>

// Error codes: phải khai báo trước SensorSnapshot
enum SensorError : uint8_t {
    SENSOR_OK = 0,
    FLAME_LEFT_OUT_OF_RANGE,
    FLAME_RIGHT_OUT_OF_RANGE,
    FLAME_SIGNAL_INVALID
};

/*
 * Current hardware has two analog flame sensors:
 * LEFT  -> PIN_FLAME_LEFT
 * RIGHT -> PIN_FLAME_RIGHT
 *
 * flameCenter and flameCenterSignal are retained for compatibility.
 * With the 2-sensor configuration, both center fields are set to 0.
 */
struct SensorSnapshot {
    uint16_t flameLeft;
    uint16_t flameCenter;
    uint16_t flameRight;

    uint16_t flameLeftSignal;
    uint16_t flameCenterSignal;
    uint16_t flameRightSignal;

    uint16_t mq2;
    uint16_t mq2Signal;

    uint16_t distanceCm;
    bool ultrasonicValid;
    uint32_t ultrasonicMs;

    uint16_t sa2DistanceCm;
    bool sa2Valid;
    uint32_t sa2Ms;

    bool mwirDetected;
    bool gasDetected;
    bool gasReady;
    bool fireDetected;
    bool fireNear;
    bool ready;

    uint32_t heartbeatMs;

    // Sensor health and error information
    SensorError lastError;
    uint32_t errorCount;
    uint8_t flameLeftHealth;
    uint8_t flameRightHealth;
    uint16_t flameLeftFlicker;
    uint16_t flameRightFlicker;
    bool flameVerifying;

    // MPU6050 relative orientation. yawDeg is integrated from the Z gyro and
    // is relative to the direction at boot; it is not an absolute compass.
    float accelXG;
    float accelYG;
    float accelZG;
    float yawDeg;
    float gyroZDps;
    bool imuValid;
    uint32_t imuMs;
};

// Bounds checking
bool isFlameAmbientValid(uint16_t ambient);

// Health checking
uint8_t getFlameLeftHealth();
uint8_t getFlameRightHealth();

// Error handling
SensorError getLastError();
void clearError();

namespace SensorManager {
    void begin();
    void update();
    SensorSnapshot get();

    uint16_t ultrasonicCm();
    uint16_t readUltrasonicNow();
    uint16_t readSA2Now();

    void setUltrasonicScanMode(bool enabled);

    bool fireConfirmed();
    int strongestFlame();
}
