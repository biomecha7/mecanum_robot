#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "RobotPins.h"

// Freenove / WS2812 7-LED circle (NeoPixel Jewel layout):
//   index 0     = center
//   indices 1-6 = ring (clockwise from DIN chain order)

enum class RgbEffect : uint8_t {
  Paint = 0,     // sticks paint hue / sat / brightness
  RainbowSpin,   // spinning rainbow; stick = speed + brightness
  Comet,         // pointer + trail around the ring
  Pulse,         // breathing glow; stick = hue + rate
  Sparkle,       // glitter; stick = density + color
  Fire,          // warm flicker; stick = intensity + coolness
  Count
};

class RgbLedDriver {
public:
  RgbLedDriver(uint8_t pin = RGB_LED_PIN, uint16_t count = RGB_LED_COUNT);

  bool begin();
  void clear();
  void setEnabled(bool on);
  bool enabled() const { return _enabled; }

  void nextEffect();
  void prevEffect();
  void setEffect(RgbEffect e);
  RgbEffect effect() const { return _effect; }
  const char* effectName() const;

  // Stick axes in [-1, 1]. Call every loop while LED mode is active.
  void update(float lx, float ly, float rx, float ry);

private:
  Adafruit_NeoPixel _strip;
  bool _enabled = false;
  RgbEffect _effect = RgbEffect::Paint;
  uint32_t _lastMs = 0;
  float _angle = 0.0f;       // radians — spin / comet phase
  float _phase = 0.0f;       // generic oscillator
  uint8_t _heat[RGB_LED_COUNT] = {};

  void showPaint(float lx, float ly, float rx, float ry);
  void showRainbowSpin(float lx, float ly, float /*rx*/, float ry, float dt);
  void showComet(float lx, float ly, float rx, float ry, float dt);
  void showPulse(float lx, float ly, float rx, float ry, float dt);
  void showSparkle(float lx, float ly, float rx, float ry, float dt);
  void showFire(float lx, float ly, float rx, float ry, float dt);

  static uint32_t hsv(uint16_t h, uint8_t s, uint8_t v);
  static float clampf(float v, float lo, float hi);
  static float lerpf(float a, float b, float t);
};
