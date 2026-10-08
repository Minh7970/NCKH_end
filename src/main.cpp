#include <Arduino.h>
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "RobotConfig.h"
#include "MotorControl.h"
#include "SensorManager.h"
#include "MapMemory.h"
#include "Navigation.h"
#include "FireControl.h"
#include "WifiMapServer.h"
#include "WebControl.h"
#include "FlameServoScanner.h"
#include "PowerManager.h"

TaskHandle_t sensorTaskHandle=nullptr;
TaskHandle_t navTaskHandle=nullptr;
TaskHandle_t safetyTaskHandle=nullptr;
TaskHandle_t mapTaskHandle=nullptr;
TaskHandle_t wifiTaskHandle=nullptr;

void TaskSensors(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    SensorManager::update();
    // Quét servo lửa cùng nhịp với đọc cảm biến; hàm tự giới hạn theo
    // FLAME_SERVO_INTERVAL_MS nên gọi thường xuyên hơn không gây hại.
    // Keep both flame servos stationary during low-power standby.
    if (!PowerManager::sleeping()) {
      FlameServoScanner::update();
      if (FireControl::active()) {
        if (FireControl::flameAlignmentComplete()) {
          // A source already in the 0..5 degree forward window is left exactly
          // where it was detected. After a body turn, use the fixed 0-degree
          // tube heading for both sensors.
          const uint8_t detectedAngle = FireControl::flameScanAngle();
          FlameServoScanner::aimScanAngle(
            detectedAngle <= FLAME_ALIGNMENT_TOLERANCE_DEG
              ? detectedAngle : PUMP_FIXED_HEADING_DEG);
        } else {
          // Rotate both flame servos in sync with the in-place chassis turn.
          FlameServoScanner::aimScanAngle(FireControl::flameTrackingAngle());
        }
      } else {
        FlameServoScanner::clearAim();
      }
    }
    vTaskDelayUntil(&last,pdMS_TO_TICKS(30));
  }
}

void TaskNavigation(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    Navigation::update();
    vTaskDelayUntil(&last,pdMS_TO_TICKS(20));
  }
}

void TaskPower(void* parameter) {
  PowerManager::taskLoop(parameter);
}

void TaskSafety(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    SensorSnapshot s=SensorManager::get();
    uint32_t now=millis();

    // Highest RTOS priority: if sensor task dies/stalls, stop all actuators.
    if(now-s.heartbeatMs > SENSOR_HEARTBEAT_TIMEOUT_MS) {
      MotorControl::stop();
      MotorControl::setPump(false);
    }

    vTaskDelayUntil(&last,pdMS_TO_TICKS(20));
  }
}

void TaskMap(void*) {
  TickType_t last=xTaskGetTickCount();
  for(;;) {
    static uint32_t lastSave=0;
    // During the first exploration, do not persist a partial map. A power
    // cycle will then restart mapping instead of incorrectly entering patrol.
    if(Navigation::mode() == Navigation::PATROL &&
       MapMemory::dirty() && millis()-lastSave>=MAP_SAVE_PERIOD_MS) {
      MapMemory::save();
      lastSave=millis();
      Serial.println(F("[EEPROM] map saved"));
      MapMemory::print(Navigation::pose());
    }
    vTaskDelayUntil(&last,pdMS_TO_TICKS(500));
  }
}

void TaskWifiServer(void*) {
  for (;;) {
    WifiMapServer::update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  MotorControl::begin();
  SensorManager::begin();
  FireControl::begin();
  WebControl::begin();
  FlameServoScanner::begin();

  MapMemory::begin();
  WifiMapServer::begin();
  PowerManager::begin();


  Serial.println(F("\n=== FIRE ROBOT / MODULAR MAPPING ==="));
  Serial.println(F("ESP32 DevKitC V4 + FreeRTOS"));
  Serial.print(F("Map: ")); Serial.print(MAP_W); Serial.print('x');
  Serial.print(MAP_H); Serial.print(F(", cell=")); Serial.print(MAP_CELL_CM);
  Serial.println(F(" cm"));
  // Serial.print(F("Target mapping wall gap="));
  // Serial.print(TARGET_WALL_GAP_CM);
  // Serial.println(F(" cm (NOT physically guaranteed by HC-SR04)"));
  // Serial.print(F("Physical stop=")); Serial.print(PHYSICAL_STOP_CM);
  // Serial.println(F(" cm"));
  // Serial.println(F("SRF04 scan servo: bounded sweep, cable-safe"));
  // Serial.print(F("SA2 SRF05 pins: TRIG=")); Serial.print(PIN_SA2_TRIG);
  // Serial.print(F(", ECHO=")); Serial.println(PIN_SA2_ECHO);

  Navigation::begin();

  if(MapMemory::isLoaded()) {
    Serial.println(F("[BOOT] EEPROM MAP FOUND -> PATROL"));
  } else {
    Serial.println(F("[BOOT] NO MAP -> AUTONOMOUS FIRST-RUN MAPPING"));
    Serial.println(F("[BOOT] Robot will drive, learn the room, then start patrol."));
  }

  xTaskCreate(TaskSafety,"Safety",4096,nullptr,3,&safetyTaskHandle);
  xTaskCreate(TaskSensors,"Sensor",4096,nullptr,2,&sensorTaskHandle);
  xTaskCreate(TaskNavigation,"Nav",4096,nullptr,1,&navTaskHandle);
  xTaskCreate(TaskMap,"Map",4096,nullptr,0,&mapTaskHandle);
  xTaskCreate(TaskWifiServer,"WifiMap",6144,nullptr,1,&wifiTaskHandle);
  xTaskCreate(TaskPower,"Power",3072,nullptr,1,nullptr);
}

void loop() {
  // ESP32 Arduino already runs FreeRTOS; application work is in the tasks.
  vTaskDelay(portMAX_DELAY);
}
