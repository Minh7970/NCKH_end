#include <Arduino.h>
#include <esp_sleep.h>

#include "PowerManager.h"
#include "RobotConfig.h"
#include "SensorManager.h"
#include "MotorControl.h"
#include "ServoScanner.h"
#include "WebControl.h"

namespace {
  volatile bool sleepState = false;
  volatile bool gasWakeState = false;
  volatile bool coverageSleepState = false;
}

namespace PowerManager {
  void begin() {
    sleepState = MQ2_SENSOR_ENABLED && MQ2_SLEEP_MODE_ENABLED;
    gasWakeState = false;
    coverageSleepState = false;
    if (sleepState) {
      Serial.println(F("[POWER] MQ-2 standby enabled; waiting for gas alarm"));
    }
  }

  bool sleeping() {
    return sleepState;
  }

  void enterCoverageSleep() {
    if (coverageSleepState) return;

    MotorControl::stop();
    MotorControl::setPump(false);
    ServoScanner::stop();
    gasWakeState = false;
    coverageSleepState = true;
    sleepState = true;
    Serial.println(F("[POWER] Coverage complete; entering light sleep"));
  }

  bool coverageSleepActive() {
    return coverageSleepState;
  }

  bool gasWakeActive() {
    return gasWakeState;
  }

  void taskLoop(void*) {
    TickType_t last = xTaskGetTickCount();

    for (;;) {
      if (!sleepState) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(250));
        continue;
      }

      // Safety first: no motor, pump, or radar scan is allowed while waiting.
      MotorControl::stop();
      MotorControl::setPump(false);
      ServoScanner::stop();

      // Wake briefly so safety/telemetry tasks can refresh. Coverage sleep is
      // independent of MQ-2, therefore it also works without a gas sensor.
      const uint32_t sleepIntervalMs = coverageSleepState
        ? COVERAGE_SLEEP_INTERVAL_MS : MQ2_SLEEP_INTERVAL_MS;
      esp_sleep_enable_timer_wakeup(
        (uint64_t)sleepIntervalMs * 1000ULL);
      esp_light_sleep_start();
      vTaskDelay(pdMS_TO_TICKS(80));

      const SensorSnapshot snapshot = SensorManager::get();
      if (MQ2_SENSOR_ENABLED && snapshot.gasReady && snapshot.gasDetected) {
        sleepState = false;
        gasWakeState = true;
        coverageSleepState = false;
        // A gas alarm always starts an autonomous patrol, even if the last
        // user session had left the controller in MANUAL mode.
        if (!WebControl::automaticMode()) WebControl::setMode("AUTO");
        Serial.printf("[POWER] MQ-2 alarm confirmed signal=%u; robot awake\n",
                      snapshot.mq2Signal);
        // Navigation is gated by sleeping(); its next 20 ms update starts
        // patrol/fire scanning automatically.
      }
    }
  }
}
