#include "RgbLedDriver.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

RgbLedDriver::RgbLedDriver(uint8_t pin, uint16_t count)
  : _strip(count, pin, NEO_GRB + NEO_KHZ800) {}

bool RgbLedDriver::begin() {
  if (!_strip.begin()) return false;
  _strip.setBrightness(64);
  _strip.clear();
  _strip.show();
  _lastMs = millis();
  return true;
}

void RgbLedDriver::clear() {
  _strip.clear();
  _strip.show();
}

void RgbLedDriver::setEnabled(bool on) {
  _enabled = on;
  if (!on) clear();
  else _lastMs = millis();
}

void RgbLedDriver::nextEffect() {
  uint8_t i = (uint8_t)_effect + 1;
  if (i >= (uint8_t)RgbEffect::Count) i = 0;
  setEffect((RgbEffect)i);
}

void RgbLedDriver::prevEffect() {
  int i = (int)_effect - 1;
  if (i < 0) i = (int)RgbEffect::Count - 1;
  setEffect((RgbEffect)i);
}

void RgbLedDriver::setEffect(RgbEffect e) {
  _effect = e;
  _angle = 0.0f;
  _phase = 0.0f;
  for (int i = 0; i < RGB_LED_COUNT; i++) _heat[i] = 0;
}

const char* RgbLedDriver::effectName() const {
  switch (_effect) {
    case RgbEffect::Paint:       return "Paint";
    case RgbEffect::RainbowSpin: return "RainbowSpin";
    case RgbEffect::Comet:       return "Comet";
    case RgbEffect::Pulse:       return "Pulse";
    case RgbEffect::Sparkle:     return "Sparkle";
    case RgbEffect::Fire:        return "Fire";
    default:                     return "?";
  }
}

float RgbLedDriver::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float RgbLedDriver::lerpf(float a, float b, float t) {
  return a + (b - a) * t;
}

// h: 0-65535, s/v: 0-255 → packed GRB-ready RGB for NeoPixel
uint32_t RgbLedDriver::hsv(uint16_t h, uint8_t s, uint8_t v) {
  if (s == 0) return ((uint32_t)v << 16) | ((uint32_t)v << 8) | v;

  uint8_t region = h / 10923;          // 65536/6
  uint16_t rem = (h % 10923) * 6;
  uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
  uint8_t q = (uint8_t)((v * (255 - ((s * rem) >> 16))) >> 8);
  uint8_t t = (uint8_t)((v * (255 - ((s * (65535 - rem)) >> 16))) >> 8);

  uint8_t r, g, b;
  switch (region) {
    case 0:  r = v; g = t; b = p; break;
    case 1:  r = q; g = v; b = p; break;
    case 2:  r = p; g = v; b = t; break;
    case 3:  r = p; g = q; b = v; break;
    case 4:  r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void RgbLedDriver::update(float lx, float ly, float rx, float ry) {
  if (!_enabled) return;

  uint32_t now = millis();
  float dt = (now - _lastMs) / 1000.0f;
  if (dt < 0.0f) dt = 0.0f;
  if (dt > 0.05f) dt = 0.05f;
  _lastMs = now;

  switch (_effect) {
    case RgbEffect::Paint:       showPaint(lx, ly, rx, ry); break;
    case RgbEffect::RainbowSpin: showRainbowSpin(lx, ly, rx, ry, dt); break;
    case RgbEffect::Comet:       showComet(lx, ly, rx, ry, dt); break;
    case RgbEffect::Pulse:       showPulse(lx, ly, rx, ry, dt); break;
    case RgbEffect::Sparkle:     showSparkle(lx, ly, rx, ry, dt); break;
    case RgbEffect::Fire:        showFire(lx, ly, rx, ry, dt); break;
    default: break;
  }
  _strip.show();
}

void RgbLedDriver::showPaint(float lx, float ly, float rx, float ry) {
  // Left stick: hue (X) + brightness (Y up = brighter)
  // Right stick: saturation (X) + ring wash vs center accent (Y)
  float hue01 = (lx + 1.0f) * 0.5f;
  uint16_t hue = (uint16_t)(hue01 * 65535.0f);
  uint8_t bri = (uint8_t)(clampf((ly + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);
  uint8_t sat = (uint8_t)(clampf((rx + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);
  float centerMix = clampf((ry + 1.0f) * 0.5f, 0.0f, 1.0f);

  uint32_t base = hsv(hue, sat, bri);
  uint16_t accentHue = (uint16_t)(hue + 10922);  // +60°
  uint32_t accent = hsv(accentHue, sat, bri);

  _strip.setPixelColor(0, (centerMix > 0.55f) ? accent : base);
  for (int i = 1; i < RGB_LED_COUNT; i++) {
    float t = (float)(i - 1) / 6.0f;
    uint16_t h = (uint16_t)(hue + (uint16_t)(t * 8000.0f * (1.0f - centerMix)));
    _strip.setPixelColor(i, hsv(h, sat, bri));
  }
}

void RgbLedDriver::showRainbowSpin(float lx, float ly, float /*rx*/, float ry, float dt) {
  float speed = lx * 4.0f;  // rad/s, left/right spin
  if (fabsf(speed) < 0.15f) speed = 0.8f;  // idle slow spin
  _angle += speed * dt;
  if (_angle > 2.0f * PI) _angle -= 2.0f * PI;
  if (_angle < 0.0f) _angle += 2.0f * PI;

  uint8_t bri = (uint8_t)(clampf((ly + 1.0f) * 0.5f, 0.05f, 1.0f) * 220.0f);
  float spread = lerpf(0.6f, 2.2f, clampf((ry + 1.0f) * 0.5f, 0.0f, 1.0f));

  // Soft white center as "hub"
  _strip.setPixelColor(0, hsv(0, 0, (uint8_t)(bri * 0.45f)));

  for (int i = 0; i < 6; i++) {
    float a = _angle + (float)i * (2.0f * PI / 6.0f);
    uint16_t hue = (uint16_t)((a / (2.0f * PI)) * 65535.0f * spread);
    _strip.setPixelColor(i + 1, hsv(hue, 255, bri));
  }
}

void RgbLedDriver::showComet(float lx, float ly, float rx, float ry, float dt) {
  // Stick vector aims the comet head; magnitude = brightness.
  // Right X adds auto-spin; right Y = trail length.
  float mag = sqrtf(lx * lx + ly * ly);
  if (mag > 0.12f) {
    _angle = atan2f(lx, ly);  // 0 = stick up
  } else {
    _angle += (0.6f + rx * 3.5f) * dt;
  }
  while (_angle > 2.0f * PI) _angle -= 2.0f * PI;
  while (_angle < 0.0f) _angle += 2.0f * PI;

  uint8_t bri = (uint8_t)(clampf(mag > 0.12f ? mag : 0.55f, 0.1f, 1.0f) * 255.0f);
  float trail = lerpf(1.0f, 4.5f, clampf((ry + 1.0f) * 0.5f, 0.0f, 1.0f));
  uint16_t hue = (uint16_t)(clampf((rx + 1.0f) * 0.5f, 0.0f, 1.0f) * 65535.0f);

  _strip.clear();
  // Dim hub
  _strip.setPixelColor(0, hsv(hue, 180, (uint8_t)(bri * 0.25f)));

  float head = (_angle / (2.0f * PI)) * 6.0f;  // 0..6 on ring
  for (int i = 0; i < 6; i++) {
    float d = fabsf((float)i - head);
    if (d > 3.0f) d = 6.0f - d;
    float fall = clampf(1.0f - d / trail, 0.0f, 1.0f);
    fall = fall * fall;
    uint8_t v = (uint8_t)(bri * fall);
    if (v > 0) _strip.setPixelColor(i + 1, hsv(hue, 255, v));
  }
}

void RgbLedDriver::showPulse(float lx, float ly, float rx, float ry, float dt) {
  float rate = lerpf(0.4f, 4.0f, clampf((lx + 1.0f) * 0.5f, 0.0f, 1.0f));
  _phase += rate * dt * 2.0f * PI;
  if (_phase > 2.0f * PI) _phase -= 2.0f * PI;

  float breath = 0.5f + 0.5f * sinf(_phase);
  uint8_t bri = (uint8_t)(clampf((ly + 1.0f) * 0.5f, 0.05f, 1.0f) * breath * 255.0f);
  uint16_t hue = (uint16_t)(clampf((rx + 1.0f) * 0.5f, 0.0f, 1.0f) * 65535.0f);
  uint8_t sat = (uint8_t)(clampf((ry + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);

  // Center peaks slightly ahead of the ring for a "heartbeat" feel
  float centerBreath = 0.5f + 0.5f * sinf(_phase + 0.45f);
  uint8_t cb = (uint8_t)(clampf((ly + 1.0f) * 0.5f, 0.05f, 1.0f) * centerBreath * 255.0f);

  _strip.setPixelColor(0, hsv(hue, sat, cb));
  for (int i = 1; i < RGB_LED_COUNT; i++) {
    _strip.setPixelColor(i, hsv(hue, sat, bri));
  }
}

void RgbLedDriver::showSparkle(float lx, float ly, float rx, float ry, float dt) {
  (void)dt;
  // Fade existing pixels
  for (int i = 0; i < RGB_LED_COUNT; i++) {
    uint32_t c = _strip.getPixelColor(i);
    uint8_t r = (c >> 16) & 0xFF;
    uint8_t g = (c >> 8) & 0xFF;
    uint8_t b = c & 0xFF;
    r = (uint8_t)(r * 0.78f);
    g = (uint8_t)(g * 0.78f);
    b = (uint8_t)(b * 0.78f);
    _strip.setPixelColor(i, ((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
  }

  float density = clampf((lx + 1.0f) * 0.5f, 0.0f, 1.0f);
  int sparks = (int)(density * 3.0f + 0.5f);
  uint8_t bri = (uint8_t)(clampf((ly + 1.0f) * 0.5f, 0.15f, 1.0f) * 255.0f);
  uint16_t hue = (uint16_t)(clampf((rx + 1.0f) * 0.5f, 0.0f, 1.0f) * 65535.0f);
  uint8_t sat = (uint8_t)(clampf((ry + 1.0f) * 0.5f, 0.0f, 1.0f) * 255.0f);

  for (int s = 0; s < sparks; s++) {
    if ((random(100) < 45)) {
      int idx = random(RGB_LED_COUNT);
      uint16_t h = hue + (uint16_t)random(4000);
      _strip.setPixelColor(idx, hsv(h, sat, bri));
    }
  }
}

void RgbLedDriver::showFire(float lx, float ly, float rx, float ry, float dt) {
  (void)dt;
  float intensity = clampf((ly + 1.0f) * 0.5f, 0.1f, 1.0f);
  int cooling = (int)lerpf(40.0f, 12.0f, intensity);
  int sparking = (int)lerpf(40.0f, 160.0f, clampf((lx + 1.0f) * 0.5f, 0.0f, 1.0f));
  float coolHue = clampf((rx + 1.0f) * 0.5f, 0.0f, 1.0f);  // 0=red/orange, 1=blue fire
  float wind = rx;  // bias sparks around the ring

  // Cool all cells a little
  for (int i = 0; i < RGB_LED_COUNT; i++) {
    int c = _heat[i] - random(cooling / 2, cooling + 1);
    _heat[i] = (c < 0) ? 0 : (uint8_t)c;
  }

  // Heat drifts toward center from ring, and around the ring
  for (int i = 6; i >= 2; i--) {
    _heat[i] = (uint8_t)((_heat[i] + _heat[i - 1] + _heat[i - 2]) / 3);
  }
  // Center averages ring
  uint16_t sum = 0;
  for (int i = 1; i < RGB_LED_COUNT; i++) sum += _heat[i];
  _heat[0] = (uint8_t)(sum / 6);

  // Random sparks on ring
  if (random(255) < sparking) {
    int base = 1 + random(6);
    int bias = (int)(wind * 2.0f);
    int idx = 1 + ((base - 1 + bias + 12) % 6);
    int h = _heat[idx] + random(160, 255);
    _heat[idx] = (h > 255) ? 255 : (uint8_t)h;
  }

  uint8_t briScale = (uint8_t)(clampf((ry + 1.0f) * 0.5f, 0.2f, 1.0f) * 255.0f);

  for (int i = 0; i < RGB_LED_COUNT; i++) {
    uint8_t t = _heat[i];
    // Map heat → color. Warm: red→orange→yellow. Cool: purple→blue→cyan.
    uint16_t hue;
    uint8_t sat;
    if (coolHue < 0.5f) {
      float w = coolHue * 2.0f;
      hue = (uint16_t)lerpf(0.0f, 7000.0f, (t / 255.0f));  // red→yellow
      hue = (uint16_t)lerpf((float)hue, (float)(hue + 2000), w);
      sat = 255;
    } else {
      float w = (coolHue - 0.5f) * 2.0f;
      hue = (uint16_t)lerpf(48000.0f, 36000.0f, t / 255.0f);  // blue→cyan
      hue = (uint16_t)lerpf(52000.0f, (float)hue, w);
      sat = (uint8_t)lerpf(200.0f, 255.0f, t / 255.0f);
    }
    uint8_t v = (uint8_t)((t * briScale) / 255);
    _strip.setPixelColor(i, hsv(hue, sat, v));
  }
}
