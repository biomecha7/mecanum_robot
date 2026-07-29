#pragma once
#include <Arduino.h>

namespace Oled {
  void begin();
  void setStatus(const char* status);
  void setMotorsActive(bool active);
  /** Re-assert Vext; paints only when idle and status changed. */
  void service();
}
