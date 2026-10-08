#pragma once
#include <Arduino.h>

namespace ServoScanner {
  void begin();
  void startScan();
  void update();
  bool active();
  bool measurementDue();
  uint8_t angle();
  void measurementTaken();
  bool cycleFinished();
  void stop();
}
