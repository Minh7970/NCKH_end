#pragma once

#include <Arduino.h>

namespace MotorControl {

  void begin();

  // Basic differential-drive commands.
  void forward(int pwm);
  // Positive correction speeds up the left side and slows the right side,
  // producing a small clockwise/right correction while moving forward.
  void forwardCorrected(int pwm, int correction);
  void backward(int pwm);
  void left(int pwm);
  void right(int pwm);
  void stop();

  // Pump relay is kept here because the existing Navigation API uses it.
  void setPump(bool on);

}
