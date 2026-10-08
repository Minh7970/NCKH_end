#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include "WebControl.h"
#include "MotorControl.h"
#include "RobotConfig.h"
#include "SensorManager.h"
#include "PowerManager.h"

// Namespace ẩn danh: trạng thái điều khiển từ web chỉ dùng nội bộ trong file này
namespace {
  // Thời gian tối đa một lệnh thủ công còn hiệu lực (ms). Nếu web ngừng gửi lệnh
  // (mất kết nối, đóng trình duyệt) quá thời gian này thì robot tự dừng
  constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 1200;

  // Khóa (spinlock) FreeRTOS bảo vệ các biến điều khiển, vì chúng được ghi bởi
  // tác vụ web server và được đọc bởi tác vụ điều khiển robot
  portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;
  // Chế độ điều khiển hiện tại: tự động hoặc thủ công (mặc định là tự động)
  WebControl::Mode controlMode = WebControl::AUTO_MODE;
  // Lệnh lái hiện tại nhận từ web (mặc định là dừng)
  WebControl::DriveCommand driveCommand = WebControl::STOP_COMMAND;
  // Tốc độ PWM dùng cho lệnh thủ công (0..255)
  uint8_t driveSpeed = 150;
  // Thời điểm (ms) mà lệnh hiện tại hết hiệu lực
  uint32_t commandExpiresMs = 0;
}

namespace WebControl {
  // Khởi tạo: đặt về chế độ tự động, dừng xe, tốc độ mặc định
  void begin() {
    taskENTER_CRITICAL(&controlMux);
    controlMode = AUTO_MODE;
    driveCommand = STOP_COMMAND;
    driveSpeed = MOTOR_SPEED_MANUAL;
    commandExpiresMs = 0;
    taskEXIT_CRITICAL(&controlMux);
  }

  // Trả về chế độ điều khiển hiện tại (đọc trong vùng găng để an toàn giữa các tác vụ)
  Mode mode() {
    taskENTER_CRITICAL(&controlMux);
    Mode result = controlMode;
    taskEXIT_CRITICAL(&controlMux);
    return result;
  }

  // Trả về true nếu đang ở chế độ tự động
  bool automaticMode() {
    return mode() == AUTO_MODE;
  }

  // Trả về tên chế độ dạng chuỗi (dùng để hiển thị trên giao diện web)
  const char* modeName() {
    return automaticMode() ? "AUTO" : "MANUAL";
  }

  // Trả về tên lệnh lái hiện tại dạng chuỗi
  const char* commandName() {
    // Sao chép lệnh ra biến cục bộ trong vùng găng rồi mới xử lý bên ngoài
    taskENTER_CRITICAL(&controlMux);
    DriveCommand command = driveCommand;
    taskEXIT_CRITICAL(&controlMux);
    switch (command) {
      case FORWARD_COMMAND: return "FORWARD";
      case BACKWARD_COMMAND: return "BACKWARD";
      case LEFT_COMMAND: return "LEFT";
      case RIGHT_COMMAND: return "RIGHT";
      case FORWARD_LEFT_COMMAND:   return "FORWARD_LEFT";
      case FORWARD_RIGHT_COMMAND:  return "FORWARD_RIGHT";
      case BACKWARD_LEFT_COMMAND:  return "BACKWARD_LEFT";
      case BACKWARD_RIGHT_COMMAND: return "BACKWARD_RIGHT";
      default: return "STOP";
    }
  }

  // Trả về tốc độ PWM đang đặt cho chế độ thủ công
  uint8_t speed() {
    taskENTER_CRITICAL(&controlMux);
    uint8_t result = driveSpeed;
    taskEXIT_CRITICAL(&controlMux);
    return result;
  }

  // Đổi chế độ điều khiển theo chuỗi "AUTO" hoặc "MANUAL" (không phân biệt hoa thường)
  // Trả về false nếu chuỗi không hợp lệ
  bool setMode(const String& value) {
    Mode requested;
    if (value.equalsIgnoreCase("AUTO")) requested = AUTO_MODE;
    else if (value.equalsIgnoreCase("MANUAL")) requested = MANUAL_MODE;
    else return false;

    // Đổi chế độ và xóa lệnh cũ để tránh robot chạy tiếp theo lệnh trước đó
    taskENTER_CRITICAL(&controlMux);
    controlMode = requested;
    driveCommand = STOP_COMMAND;
    commandExpiresMs = 0;
    taskEXIT_CRITICAL(&controlMux);
    // Dừng động cơ ngay khi đổi chế độ (gọi ngoài vùng găng)
    MotorControl::stop();
    return true;
  }

  // Nhận lệnh lái thủ công từ web kèm tốc độ yêu cầu; trả về false nếu lệnh không hợp lệ
  // hoặc robot không ở chế độ thủ công
  bool setDrive(const String& command, int requestedSpeed) {
    // Chuyển chuỗi lệnh sang giá trị enum
    DriveCommand requested;
    if (command.equalsIgnoreCase("FORWARD")) requested = FORWARD_COMMAND;
    else if (command.equalsIgnoreCase("BACKWARD")) requested = BACKWARD_COMMAND;
    else if (command.equalsIgnoreCase("LEFT")) requested = LEFT_COMMAND;
    else if (command.equalsIgnoreCase("RIGHT")) requested = RIGHT_COMMAND;
    else if (command.equalsIgnoreCase("FORWARD_LEFT")) requested = FORWARD_LEFT_COMMAND;
    else if (command.equalsIgnoreCase("FORWARD_RIGHT")) requested = FORWARD_RIGHT_COMMAND;
    else if (command.equalsIgnoreCase("BACKWARD_LEFT")) requested = BACKWARD_LEFT_COMMAND;
    else if (command.equalsIgnoreCase("BACKWARD_RIGHT")) requested = BACKWARD_RIGHT_COMMAND;
    else if (command.equalsIgnoreCase("STOP")) requested = STOP_COMMAND;
    else return false;

    taskENTER_CRITICAL(&controlMux);
    // Không ở chế độ thủ công thì từ chối lệnh (nhớ thoát vùng găng trước khi return)
    if (controlMode != MANUAL_MODE) {
      taskEXIT_CRITICAL(&controlMux);
      return false;
    }
    driveCommand = requested;
    // Giới hạn tốc độ trong khoảng 0..255
    driveSpeed = (uint8_t)constrain(requestedSpeed, 0, 255);
    // Đặt hạn hiệu lực: web phải gửi lại lệnh liên tục để robot tiếp tục chạy
    commandExpiresMs = millis() + MANUAL_COMMAND_TIMEOUT_MS;
    taskEXIT_CRITICAL(&controlMux);
    return true;
  }

  // Dừng khẩn cấp: xóa lệnh hiện tại và dừng động cơ
  void stop() {
    taskENTER_CRITICAL(&controlMux);
    driveCommand = STOP_COMMAND;
    commandExpiresMs = 0;
    taskEXIT_CRITICAL(&controlMux);
    MotorControl::stop();
  }

  // Hàm cập nhật chính, gọi lặp lại: biến lệnh thủ công thành điều khiển động cơ thực tế,
  // có kiểm tra an toàn
  void update() {
    if (PowerManager::sleeping()) {
      MotorControl::stop();
      return;
    }
    // Lấy bản sao các biến điều khiển trong một vùng găng duy nhất để chúng nhất quán
    Mode currentMode;
    DriveCommand currentCommand;
    uint8_t currentSpeed;
    uint32_t expires;
    taskENTER_CRITICAL(&controlMux);
    currentMode = controlMode;
    currentCommand = driveCommand;
    currentSpeed = driveSpeed;
    expires = commandExpiresMs;
    taskEXIT_CRITICAL(&controlMux);

    // Chế độ tự động thì module Navigation lo việc lái, ở đây không làm gì
    if (currentMode != MANUAL_MODE) return;

    const SensorSnapshot sensors = SensorManager::get();
    const uint32_t now = millis();
    // Lệnh đã hết hạn (web ngừng gửi lệnh): chuyển về STOP
    if (now > expires && currentCommand != STOP_COMMAND) {
      taskENTER_CRITICAL(&controlMux);
      driveCommand = STOP_COMMAND;
      commandExpiresMs = 0;
      taskEXIT_CRITICAL(&controlMux);
      currentCommand = STOP_COMMAND;
    }

    // Các điều kiện buộc dừng: cảm biến chưa sẵn sàng, cảm biến ngừng cập nhật (mất heartbeat),
    // đang có cháy (FireControl/Navigation sẽ điều khiển), hoặc lệnh là STOP
    if (!sensors.ready || now - sensors.heartbeatMs > SENSOR_HEARTBEAT_TIMEOUT_MS ||
        sensors.fireDetected || currentCommand == STOP_COMMAND) {
      MotorControl::stop();
      return;
    }

    // Forward manual commands use the same 15 cm threshold and configurable
    // no-echo policy as autonomous navigation.
    const bool forwardCommand =
      currentCommand == FORWARD_COMMAND ||
      currentCommand == FORWARD_LEFT_COMMAND ||
      currentCommand == FORWARD_RIGHT_COMMAND;
    const bool sa2Fresh = sensors.sa2Valid &&
      now - sensors.sa2Ms <= 2 * ULTRASONIC_PERIOD_MS + 100;
    if (forwardCommand &&
        ((SA2_STOP_ON_INVALID_READING && !sa2Fresh) ||
         (sa2Fresh && sensors.sa2DistanceCm <= FORWARD_CLEARANCE_CM))) {
      MotorControl::stop();
      return;
    }

    // Thực thi lệnh lái tương ứng với tốc độ đã đặt
    switch (currentCommand) {
      case FORWARD_COMMAND: MotorControl::forward(currentSpeed); break;
      case BACKWARD_COMMAND: MotorControl::backward(currentSpeed); break;
      case LEFT_COMMAND: MotorControl::left(currentSpeed); break;
      case RIGHT_COMMAND: MotorControl::right(currentSpeed); break;
      // case FORWARD_LEFT_COMMAND:   MotorControl::forwardLeft(currentSpeed); break;
      // case FORWARD_RIGHT_COMMAND:  MotorControl::forwardRight(currentSpeed); break;
      // case BACKWARD_LEFT_COMMAND:  MotorControl::backwardLeft(currentSpeed); break;
      // case BACKWARD_RIGHT_COMMAND: MotorControl::backwardRight(currentSpeed); break;
      default: MotorControl::stop(); break;
    }
  }
}
