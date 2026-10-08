#pragma once

#include <Arduino.h>
#include "RobotConfig.h"

namespace FireControl {
  enum Direction : uint8_t {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    FORWARD = 3,
    AMBIGUOUS = 4,
    BACKWARD = 5
  };

  void begin();
  void update(bool confirmedFire, bool nearFire, uint16_t flameLeft,
              uint16_t flameRight,
              uint8_t scanAngleDeg = FLAME_SERVO_CENTER_DEG);

  Direction direction();
  uint8_t flameScanAngle();
  uint8_t flameTrackingAngle();
  bool flameAtScanEdge();
  bool flameAlignmentComplete();
  bool active();

  bool isPumpRunning();
  uint8_t getPumpCycleCount();
  uint32_t getFireElapsedMs();
  void emergencyStop();
}
