#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "WifiMapServer.h"
#include "RobotConfig.h"
#include "MapMemory.h"
#include "Navigation.h"
#include "WebControl.h"
#include "SensorManager.h"
#include "ScanMapper.h"
#include "FireControl.h"
#include "PowerManager.h"

// Namespace ẩn danh: máy chủ web và các hàm xử lý yêu cầu HTTP chỉ dùng nội bộ trong file này
namespace {
  // Máy chủ web lắng nghe ở cổng 80 (HTTP)
  WebServer server(80);

  // Thêm các header CORS để trang web ở nguồn khác (ví dụ app giao diện) có thể gọi API,
  // và header Cache-Control để trình duyệt không lưu cache dữ liệu thời gian thực
  void addCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.sendHeader("Cache-Control", "no-store");
  }

  // Xử lý GET /api/map: trả về bản đồ lưới, vị trí robot, số đo quét mới nhất
  // và thông tin an toàn dưới dạng JSON
  void sendMap() {
    JsonDocument document;
    // Thông tin chung của bản đồ: kích thước lưới và kích thước mỗi ô (cm)
    document["width"] = MAP_W;
    document["height"] = MAP_H;
    document["cell_cm"] = MAP_CELL_CM;

    // Vị trí và hướng hiện tại của robot (heading: 0=Bắc, 1=Đông, 2=Nam, 3=Tây)
    const Pose currentPose = Navigation::pose();
    JsonObject pose = document["pose"].to<JsonObject>();
    pose["x"] = currentPose.x;
    pose["y"] = currentPose.y;
    pose["heading"] = (uint8_t)currentPose.heading;

    // Số đo quét siêu âm gần nhất (để giao diện vẽ tia radar theo thời gian thực)
    float scanAngle = 0.0f;
    uint16_t scanDistance = 0;
    bool scanValid = false;
    uint32_t scanMeasuredAt = 0;
    ScanMapper::latestReading(scanAngle, scanDistance, scanValid, scanMeasuredAt);
    JsonObject scan = document["scan"].to<JsonObject>();
    scan["angle_deg"] = scanAngle;
    scan["distance_cm"] = scanDistance;
    scan["valid"] = scanValid;
    // Tuổi của số đo (ms) tính từ lúc đo đến nay
    scan["age_ms"] = millis() - scanMeasuredAt;
    uint32_t radarCycles = 0;
    uint32_t radarMapUpdatedMs = 0;
    uint16_t radarReflections = 0;
    bool radarPublishing = false;
    ScanMapper::sweepStatus(radarCycles, radarMapUpdatedMs,
                            radarReflections, radarPublishing);
    scan["completed_cycles"] = radarCycles;
    scan["map_updated_ms"] = radarMapUpdatedMs;
    scan["confirmed_reflections"] = radarReflections;
    scan["map_publishing"] = radarPublishing;

    // Đánh giá điều kiện an toàn để cho phép đi tiến, dựa trên cảm biến SA2
    const SensorSnapshot sensors = SensorManager::get();
    const uint32_t now = millis();
    // Số đo SA2 "còn mới": hợp lệ và không quá cũ (quá 2 chu kỳ đo + 100 ms là cũ)
    const bool sa2Fresh = sensors.sa2Valid &&
      now - sensors.sa2Ms <= 2 * ULTRASONIC_PERIOD_MS + 100;
    // Cho phép đi tiến khi hệ thống sẵn sàng, không cháy và không có echo SA2 gần.
    const bool forwardAllowed = sensors.ready && !sensors.fireDetected &&
      (!SA2_STOP_ON_INVALID_READING || sa2Fresh) &&
      (!sa2Fresh || sensors.sa2DistanceCm > FORWARD_CLEARANCE_CM);
    // Lý do chặn đi tiến (theo thứ tự ưu tiên), hoặc "clear" nếu không bị chặn
    const char* forwardBlockReason = !sensors.ready ? "sensor_not_ready" :
      sensors.fireDetected ? "fire_detected" :
      SA2_STOP_ON_INVALID_READING && !sensors.sa2Valid ? "sa2_invalid" :
      SA2_STOP_ON_INVALID_READING && !sa2Fresh ? "sa2_stale" :
      sa2Fresh && sensors.sa2DistanceCm <= FORWARD_CLEARANCE_CM
        ? "obstacle_too_close" : "clear";
    // Đóng gói thông tin an toàn vào JSON
    JsonObject safety = document["safety"].to<JsonObject>();
    safety["sa2_distance_cm"] = sensors.sa2Valid ? sensors.sa2DistanceCm : 0;
    safety["sa2_valid"] = sensors.sa2Valid;
    safety["sa2_fresh"] = sa2Fresh;
    safety["forward_allowed"] = forwardAllowed;
    safety["forward_block_reason"] = forwardBlockReason;
    safety["threshold_cm"] = FORWARD_CLEARANCE_CM;

    // Chuyển lưới bản đồ thành mảng chuỗi, mỗi chuỗi là một hàng:
    // '.' ô trống, '#' vật cản, '?' chưa biết
    JsonArray rows = document["grid"].to<JsonArray>();
    for (uint8_t y = 0; y < MAP_H; ++y) {
      char row[MAP_W + 1];
      for (uint8_t x = 0; x < MAP_W; ++x) {
        const CellType cell = MapMemory::get(x, y);
        row[x] = cell == CELL_FREE ? '.' :
                 cell == CELL_OBSTACLE ? '#' : '?';
      }
      // Kết thúc chuỗi C
      row[MAP_W] = '\0';
      rows.add(row);
    }

    // Tuần tự hóa JSON thành chuỗi rồi gửi về client
    String response;
    serializeJson(document, response);
    addCorsHeaders();
    server.send(200, "application/json", response);
  }

  // Xử lý GET /api/control/status: trả về trạng thái điều khiển và an toàn (JSON)
  void sendControlStatus() {
    JsonDocument document;
    const SensorSnapshot sensors = SensorManager::get();
    const uint32_t now = millis();
    const bool heartbeatFresh = now - sensors.heartbeatMs <= SENSOR_HEARTBEAT_TIMEOUT_MS;
    // Cùng cách tính an toàn đi tiến như trong sendMap()
    const bool sa2Fresh = sensors.sa2Valid &&
      now - sensors.sa2Ms <= 2 * ULTRASONIC_PERIOD_MS + 100;
    const bool forwardAllowed = sensors.ready && !sensors.fireDetected &&
      (!SA2_STOP_ON_INVALID_READING || sa2Fresh) &&
      (!sa2Fresh || sensors.sa2DistanceCm > FORWARD_CLEARANCE_CM);
    const char* forwardBlockReason = !sensors.ready ? "sensor_not_ready" :
      sensors.fireDetected ? "fire_detected" :
      SA2_STOP_ON_INVALID_READING && !sensors.sa2Valid ? "sa2_invalid" :
      SA2_STOP_ON_INVALID_READING && !sa2Fresh ? "sa2_stale" :
      sa2Fresh && sensors.sa2DistanceCm <= FORWARD_CLEARANCE_CM
        ? "obstacle_too_close" : "clear";

    // Chế độ (AUTO/MANUAL), lệnh lái hiện tại và tốc độ
    document["mode"] = WebControl::modeName();
    document["command"] = WebControl::commandName();
    document["speed"] = WebControl::speed();
    // Trạng thái của điều hướng tự động: FIRE, PATROL hoặc EXPLORE
    document["auto_state"] = Navigation::mode() == Navigation::FIRE ? "FIRE" :
      Navigation::mode() == Navigation::PATROL ? "PATROL" : "EXPLORE";
    const bool manualAllowed = sensors.ready && heartbeatFresh && !sensors.fireDetected;
    const char* manualBlockReason = !sensors.ready ? "sensor_not_ready" :
      !heartbeatFresh ? "sensor_heartbeat_stale" :
      sensors.fireDetected ? "fire_detected" : "ready";
    document["manual_drive_allowed"] = manualAllowed;
    document["manual_block_reason"] = manualBlockReason;
    document["sensor_ready"] = sensors.ready;
    document["heartbeat_fresh"] = heartbeatFresh;
    document["imu_enabled"] = MPU6050_ENABLED;
    document["imu_valid"] = sensors.imuValid;
    document["imu_fresh"] = sensors.imuValid &&
      now - sensors.imuMs <= MPU6050_FRESH_MS;
    document["yaw_deg"] = sensors.yawDeg;
    document["gyro_z_dps"] = sensors.gyroZDps;
    document["accel_x_g"] = sensors.accelXG;
    document["accel_y_g"] = sensors.accelYG;
    document["accel_z_g"] = sensors.accelZG;
    document["fire_detected"] = sensors.fireDetected;
    document["fire_near"] = sensors.fireNear;
    document["mwir_enabled"] = MWIR_SENSOR_ENABLED;
    document["mwir_detected"] = sensors.mwirDetected;
    document["gas_enabled"] = MQ2_SENSOR_ENABLED;
    document["gas_detected"] = sensors.gasDetected;
    document["gas_ready"] = sensors.gasReady;
    document["mq2_value"] = sensors.mq2;
    document["mq2_signal"] = sensors.mq2Signal;
    document["mq2_threshold"] = MQ2_DETECT_DELTA;
    document["power_sleeping"] = PowerManager::sleeping();
    document["coverage_sleep_active"] = PowerManager::coverageSleepActive();
    document["gas_wake_active"] = PowerManager::gasWakeActive();
    document["flame_digital_mode"] = FLAME_USE_DIGITAL_OUTPUT;
    document["flame_left_do"] = FLAME_USE_DIGITAL_OUTPUT && sensors.flameLeftSignal > 0;
    document["flame_right_do"] = FLAME_USE_DIGITAL_OUTPUT && sensors.flameRightSignal > 0;
    document["flame_left_signal"] = sensors.flameLeftSignal;
    document["flame_right_signal"] = sensors.flameRightSignal;
    document["flame_left_flicker"] = sensors.flameLeftFlicker;
    document["flame_right_flicker"] = sensors.flameRightFlicker;
    document["flame_verifying"] = sensors.flameVerifying;
    document["pump_on"] = FireControl::isPumpRunning();
    document["pump_cycles"] = FireControl::getPumpCycleCount();
    document["pump_fixed_heading_deg"] = PUMP_FIXED_HEADING_DEG;
    document["flame_scan_angle"] = FireControl::flameScanAngle();
    document["flame_scan_edge"] = FireControl::flameAtScanEdge();
    document["flame_alignment_complete"] = FireControl::flameAlignmentComplete();
    // Thông tin cảm biến SA2 và điều kiện cho phép đi tiến
    document["sa2_distance_cm"] = sensors.sa2Valid ? sensors.sa2DistanceCm : 0;
    document["sa2_valid"] = sensors.sa2Valid;
    document["sa2_fresh"] = sa2Fresh;
    document["forward_allowed"] = forwardAllowed;
    document["forward_block_reason"] = forwardBlockReason;
    document["forward_threshold_cm"] = FORWARD_CLEARANCE_CM;

    String response;
    serializeJson(document, response);
    addCorsHeaders();
    server.send(200, "application/json", response);
  }

  // Xử lý GET /api/control/mode?mode=auto|manual: đổi chế độ điều khiển
  void setControlMode() {
    const String requestedMode = server.arg("mode");
    // Tham số không hợp lệ thì trả lỗi 400 (yêu cầu sai)
    if (!WebControl::setMode(requestedMode)) {
      addCorsHeaders();
      server.send(400, "application/json", "{\"error\":\"mode must be auto or manual\"}");
      return;
    }

    // Chuyển sang thủ công thì hủy mọi hoạt động tự động đang chạy của Navigation
    if (requestedMode.equalsIgnoreCase("MANUAL")) Navigation::forceStop();
    addCorsHeaders();
    // Trả về chế độ hiện tại sau khi đổi
    server.send(200, "application/json", String("{\"mode\":\"") +
      WebControl::modeName() + "\"}");
  }

  // Xử lý GET /api/control/drive?cmd=...&speed=...: nhận lệnh lái thủ công
  void setDriveCommand() {
    const String command = server.arg("cmd");
    // Tốc độ mặc định 150 nếu client không gửi tham số speed
    const int requestedSpeed = server.hasArg("speed")
      ? server.arg("speed").toInt() : MOTOR_SPEED_MANUAL;
    // Trả lỗi 409 (xung đột) nếu không ở chế độ thủ công hoặc lệnh không hợp lệ
    if (!WebControl::setDrive(command, requestedSpeed)) {
      addCorsHeaders();
      server.send(409, "application/json",
                  "{\"error\":\"select MANUAL mode and use a valid command\"}");
      return;
    }
    addCorsHeaders();
    server.send(200, "application/json", "{\"accepted\":true}");
  }

  // Xử lý GET /api/control/stop: dừng robot ngay lập tức
  void stopDrive() {
    WebControl::stop();
    addCorsHeaders();
    server.send(200, "application/json", "{\"command\":\"STOP\"}");
  }

  // Trả lời yêu cầu OPTIONS (preflight của CORS) bằng mã 204, không có nội dung
  void sendOptions() {
    addCorsHeaders();
    server.send(204, "text/plain", "");
  }

  // Xử lý GET /: trang gốc, chỉ hiển thị thông báo API đang chạy
  void sendRoot() {
    addCorsHeaders();
    server.send(200, "text/plain",
                "FireRobot map API is running. Fetch /api/map for the occupancy grid.");
  }
}

namespace WifiMapServer {
  // Khởi tạo WiFi và đăng ký các đường dẫn API của máy chủ web
  void begin() {
    // Trước tiên thử kết nối vào mạng WiFi có sẵn (chế độ Station)
    WiFi.mode(WIFI_STA);
    bool connected = false;

    // Chỉ thử kết nối nếu có cấu hình SSID
    if (WIFI_SSID[0] != '\0') {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      const uint32_t startedAt = millis();
      // Chờ kết nối cho tới khi thành công hoặc hết thời gian (vòng lặp này chặn chương trình)
      while (WiFi.status() != WL_CONNECTED && millis() - startedAt < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
      }
      connected = WiFi.status() == WL_CONNECTED;
    }

    if (connected) {
      // Kết nối thành công: in địa chỉ IP để người dùng truy cập API
      Serial.print(F("[WIFI] connected; map URL http://"));
      Serial.print(WiFi.localIP());
      Serial.println(F("/api/map"));
    } else {
      // Không kết nối được: chuyển sang phát WiFi riêng (Access Point) làm phương án dự phòng
      WiFi.mode(WIFI_AP);
      if (WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD)) {
        Serial.print(F("[WIFI] AP started; connect to "));
        Serial.print(WIFI_AP_SSID);
        Serial.print(F(" then open http://"));
        Serial.print(WiFi.softAPIP());
        Serial.println(F("/api/map"));
      } else {
        Serial.println(F("[WIFI] failed to start station and fallback AP"));
      }
    }

    // Đăng ký các đường dẫn (route) và hàm xử lý tương ứng
    server.on("/", HTTP_GET, sendRoot);
    server.on("/api/map", HTTP_GET, sendMap);
    server.on("/api/map", HTTP_OPTIONS, sendOptions);
    server.on("/api/control/status", HTTP_GET, sendControlStatus);
    server.on("/api/control/mode", HTTP_GET, setControlMode);
    server.on("/api/control/drive", HTTP_GET, setDriveCommand);
    server.on("/api/control/stop", HTTP_GET, stopDrive);
    // Các route OPTIONS để trình duyệt vượt qua bước kiểm tra CORS trước khi gọi API
    server.on("/api/control/status", HTTP_OPTIONS, sendOptions);
    server.on("/api/control/mode", HTTP_OPTIONS, sendOptions);
    server.on("/api/control/drive", HTTP_OPTIONS, sendOptions);
    server.on("/api/control/stop", HTTP_OPTIONS, sendOptions);
    // Khởi động máy chủ web
    server.begin();
  }

  // Hàm cập nhật chính, gọi lặp lại: xử lý yêu cầu HTTP đang chờ và cập nhật điều khiển thủ công
  void update() {
    server.handleClient();
    WebControl::update();
  }
}
