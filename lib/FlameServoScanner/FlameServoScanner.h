#pragma once

#include <Arduino.h>

// Quet qua lai bang 2 servo gan cam bien lua trai/phai.
// Servo phai duoc dao chieu: goc vat ly = 180 - goc logic.
namespace FlameServoScanner {
  // Gan servo vao GPIO va dua ve vi tri bat dau.
  void begin();

  // Goi lien tuc trong loop(); khong dung delay trong ham update().
  void update();

  // Goc lenh hien tai cua servo trai (0..180).
  uint8_t leftAngle();

  // Goc vat ly cua servo phai (180 - leftAngle).
  uint8_t rightAngle();

  // Trang thai attach de main code co the kiem tra loi phan cung.
  bool ready();

  // Hold both sensors at the current angle while SensorManager verifies that
  // the IR variation is flame flicker rather than a change caused by scanning.
  void setHold(bool hold);
  bool held();
  uint8_t heldAngle();

  // Direction values match FireControl::Direction without creating a header
  // cycle: 0 NONE, 1 LEFT, 2 RIGHT, 3 FORWARD, 4 AMBIGUOUS, 5 BACKWARD.
  void aimDirection(uint8_t direction);
  // Hold both flame sensors at the actual scan angle that detected the fire.
  void aimScanAngle(uint8_t scanAngle);
  void clearAim();
  bool aiming();
}
