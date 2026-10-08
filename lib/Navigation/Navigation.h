#pragma once
#include <Arduino.h>
#include "MapTypes.h"

namespace Navigation {
  enum Mode { EXPLORE, PATROL, FIRE };
  void begin();
  void firstRunRoomScan();  // Radar room scan
  void update();
  Pose pose();
  Mode mode();
  void forceStop();
}
