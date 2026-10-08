#pragma once

namespace PowerManager {
  // Initialise the power state. Call once before the RTOS tasks start.
  void begin();

  // True while actuators/navigation must remain stopped in light sleep.
  bool sleeping();

  // Enter low-power standby after every reachable FREE cell has been covered.
  void enterCoverageSleep();

  // True when sleep was requested because coverage is complete.
  bool coverageSleepActive();

  // True after an MQ-2 alarm has released the robot into patrol/fire search.
  bool gasWakeActive();

  // RTOS task body. It periodically enters ESP32 light sleep and, when MQ-2
  // hardware is enabled, releases Navigation after a confirmed gas alarm.
  void taskLoop(void* parameter);
}
