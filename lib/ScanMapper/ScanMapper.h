#pragma once
#include <Arduino.h>
#include "MapTypes.h"

namespace ScanMapper {
  void beginSweep();
  void addReading(const Pose& pose, float worldAngleDeg, uint16_t distanceCm,
                 uint8_t validSamples, uint8_t totalSamples,
                 bool integrateIntoMap = true);
  void latestReading(float& worldAngleDeg, uint16_t& distanceCm,
                     bool& valid, uint32_t& measuredAtMs);
  void sweepStatus(uint32_t& completedSweeps, uint32_t& completedAtMs,
                   uint16_t& reflectionCount, bool& publishing);
  void finishSweep(const Pose& pose, uint8_t cycle);
}
