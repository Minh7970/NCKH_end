#pragma once
#include <Arduino.h>

namespace WebControl {
  enum Mode : uint8_t { AUTO_MODE, MANUAL_MODE };
  enum DriveCommand { STOP_COMMAND, FORWARD_COMMAND, BACKWARD_COMMAND, LEFT_COMMAND, RIGHT_COMMAND,
                    FORWARD_LEFT_COMMAND, FORWARD_RIGHT_COMMAND,
                    BACKWARD_LEFT_COMMAND, BACKWARD_RIGHT_COMMAND };
  void begin();
  Mode mode();
  bool automaticMode();
  const char* modeName();
  const char* commandName();
  uint8_t speed();
  bool setMode(const String& value);
  bool setDrive(const String& command, int requestedSpeed);
  void stop();
  void update();
}
