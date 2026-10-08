#include <Arduino.h>
#include <ESP32Servo.h>

#include "FlameServoScanner.h"
#include "RobotConfig.h"

namespace {
  Servo servoLeft;
  Servo servoRight;

  int angle = FLAME_SERVO_MIN_DEG;
  int direction = 1;
  int leftCommand = FLAME_SERVO_MIN_DEG;
  int rightCommand = FLAME_SERVO_MAX_DEG;
  uint32_t lastMoveMs = 0;
  bool scannerReady = false;
  bool holdPosition = false;
  bool aimOverride = false;
  uint8_t heldScanAngle = FLAME_SERVO_MIN_DEG;

  int clampAngle(int value) {
    if (value < 0) return 0;
    if (value > 180) return 180;
    return value;
  }

  void writeCurrentAngles() {
    const int left = clampAngle(angle);
    const int right = 180 - left;  // Dao chieu servo phai.
    leftCommand = left;
    rightCommand = clampAngle(right);

    if (servoLeft.attached()) servoLeft.write(leftCommand);
    if (servoRight.attached()) servoRight.write(rightCommand);
  }

  void writeAimAngles(int left, int right) {
    leftCommand = clampAngle(left);
    rightCommand = clampAngle(right);
    if (servoLeft.attached()) servoLeft.write(leftCommand);
    if (servoRight.attached()) servoRight.write(rightCommand);
  }
}

namespace FlameServoScanner {
  void begin() {
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    servoLeft.setPeriodHertz(50);
    servoRight.setPeriodHertz(50);

    const int leftAttach = servoLeft.attach(PIN_FLAME_SERVO_LEFT, 500, 2400);
    const int rightAttach = servoRight.attach(PIN_FLAME_SERVO_RIGHT, 500, 2400);

    scannerReady = servoLeft.attached() && servoRight.attached();

    angle = clampAngle(FLAME_SERVO_MIN_DEG);
    direction = 1;
    holdPosition = false;
    aimOverride = false;
    writeCurrentAngles();
    lastMoveMs = millis();

    Serial.printf(
      "[FLAME SERVO] LEFT GPIO=%d attach=%d attached=%s | RIGHT GPIO=%d attach=%d attached=%s\n",
      PIN_FLAME_SERVO_LEFT, leftAttach, servoLeft.attached() ? "YES" : "NO",
      PIN_FLAME_SERVO_RIGHT, rightAttach, servoRight.attached() ? "YES" : "NO");

    if (!scannerReady) {
      Serial.println("[FLAME SERVO][ERROR] Servo attach failed; check GPIO and power.");
    } else {
      Serial.println("[FLAME SERVO] Scanner started; right servo is reversed.");
    }
  }

  void update() {
    if (!scannerReady || holdPosition || aimOverride) return;

    const uint32_t now = millis();
    if ((uint32_t)(now - lastMoveMs) < FLAME_SERVO_INTERVAL_MS) return;
    lastMoveMs = now;

    // Tinh goc tiep theo truoc, sau do cap nhat va ghi dung goc hien tai.
    int nextAngle = angle + direction * FLAME_SERVO_STEP_DEG;

    if (nextAngle >= FLAME_SERVO_MAX_DEG) {
      nextAngle = FLAME_SERVO_MAX_DEG;
      direction = -1;
    } else if (nextAngle <= FLAME_SERVO_MIN_DEG) {
      nextAngle = FLAME_SERVO_MIN_DEG;
      direction = 1;
    }

    angle = clampAngle(nextAngle);
    writeCurrentAngles();
  }

  uint8_t leftAngle() {
    return (uint8_t)leftCommand;
  }

  uint8_t rightAngle() {
    return (uint8_t)rightCommand;
  }

  bool ready() {
    return scannerReady;
  }

  void setHold(bool hold) {
    if (holdPosition == hold) return;
    if (hold) heldScanAngle = (uint8_t)clampAngle(angle);
    holdPosition = hold;
    // After release, wait one full movement interval before changing angle.
    lastMoveMs = millis();
  }

  bool held() {
    return holdPosition;
  }

  uint8_t heldAngle() {
    return heldScanAngle;
  }

  void aimDirection(uint8_t targetDirection) {
    if (!scannerReady) return;

    int left = FLAME_AIM_FRONT_DEG;
    int right = FLAME_AIM_FRONT_DEG;
    if (targetDirection == 1) {
      // Only the left sensor tracks a left-side fire; the other returns to
      // center so both readings remain useful for direction confirmation.
      left = FLAME_AIM_LEFT_DEG;
    } else if (targetDirection == 2) {
      // Mirror of the left-side case.
      right = FLAME_AIM_RIGHT_DEG;
    } else if (targetDirection == 5) {
      // The right servo is mechanically mirrored during scanning, so use the
      // mirrored command when both sensors are sent to the rear direction.
      left = FLAME_AIM_REAR_DEG;
      right = 180 - FLAME_AIM_REAR_DEG;
    }

    aimOverride = true;
    holdPosition = true;
    writeAimAngles(left, right);
  }

  void aimScanAngle(uint8_t scanAngle) {
    if (!scannerReady) return;
    const int logicAngle = clampAngle(scanAngle);
    aimOverride = true;
    holdPosition = true;
    angle = logicAngle;
    direction = 1;
    writeCurrentAngles();
  }

  void clearAim() {
    if (!aimOverride) return;
    aimOverride = false;
    holdPosition = false;
    angle = FLAME_SERVO_CENTER_DEG;
    direction = 1;
    writeCurrentAngles();
    lastMoveMs = millis();
  }

  bool aiming() {
    return aimOverride;
  }
}
