#include <Arduino.h>
#include <math.h>
#include <SSD1306Wire.h>
#include <driver/gpio.h>
#include <PSX.h>
#include "RobotPins.h"
#include "MotorDriver.h"
#include "RgbLedDriver.h"

static MotorDriver* m1 = nullptr;
static MotorDriver* m2 = nullptr;
static MotorDriver* m3 = nullptr;
static MotorDriver* m4 = nullptr;
static RgbLedDriver rgbLed;

PSX psx;
static uint16_t buttons = 0, prevButtons = 0;
static float vx = 0, vy = 0, wz = 0;
static float stickLX = 0, stickLY = 0, stickRX = 0, stickRY = 0;
static float speedScale = 0.70f;
static bool estop = false;
static bool armed = false;
static bool ledMode = false;  // SELECT toggles: joystick drives the RGB circle
static bool leftEyeOn = false;
static bool rightEyeOn = false;
static uint32_t estopClearStart = 0;
static uint32_t lastPs2Ok = 0;

// Non-blocking eye animations
enum class EyeMode { Idle, WinkOnce, Show };
static EyeMode eyeMode = EyeMode::Idle;
static uint32_t eyeStepAt = 0;
static int eyeStep = 0;
static bool eyeWinkLeft = false;  // single-wink: which eye closes

struct EyeCue {
  uint16_t ms;
  bool left;
  bool right;
};

static const float DEADBAND = 0.10f;

static TaskHandle_t buzzerTaskHandle = nullptr;
static volatile bool buzzerBusy = false;

// Kid-friendly note frequencies (Hz). 0 = rest.
enum {
  NOTE_REST = 0,
  NOTE_C4 = 262, NOTE_D4 = 294, NOTE_E4 = 330, NOTE_F4 = 349,
  NOTE_G4 = 392, NOTE_A4 = 440, NOTE_B4 = 494,
  NOTE_C5 = 523, NOTE_D5 = 587, NOTE_E5 = 659, NOTE_F5 = 698,
  NOTE_G5 = 784, NOTE_A5 = 880, NOTE_B5 = 988,
  NOTE_C6 = 1047, NOTE_D6 = 1175, NOTE_E6 = 1319, NOTE_G6 = 1568,
};

struct BuzzNote { uint16_t hz; uint16_t ms; };

static void buzzerIdlePins() {
  digitalWrite(BUZZER_A, LOW);
  digitalWrite(BUZZER_B, LOW);
}

static void playToneHz(int hz, int ms) {
  if (ms <= 0) return;
  if (hz <= 0) {
    buzzerIdlePins();
    vTaskDelay(pdMS_TO_TICKS(ms));
    return;
  }

  // Differential square wave on A/B — louder on a piezo than single-ended
  const uint32_t halfUs = 1000000UL / (uint32_t)(2 * hz);
  const uint32_t endMs = millis() + (uint32_t)ms;
  bool high = false;
  while ((int32_t)(millis() - endMs) < 0) {
    high = !high;
    digitalWrite(BUZZER_A, high ? HIGH : LOW);
    digitalWrite(BUZZER_B, high ? LOW : HIGH);
    delayMicroseconds(halfUs);
  }
  buzzerIdlePins();
}

static void playTune(const BuzzNote* notes, int count) {
  for (int i = 0; i < count; i++) {
    playToneHz(notes[i].hz, notes[i].ms);
    // tiny gap so notes don't smear together
    if (notes[i].hz > 0) {
      buzzerIdlePins();
      delayMicroseconds(8000);
    }
  }
  buzzerIdlePins();
}

// Cartoon power-up → laser zap → happy arpeggio (~1.2s)
static void playFunKidSound() {
  static const BuzzNote tune[] = {
    // Rising "charge"
    {NOTE_C5, 70}, {NOTE_E5, 70}, {NOTE_G5, 70}, {NOTE_C6, 90},
    // Boing
    {NOTE_G6, 50}, {NOTE_E6, 50}, {NOTE_C6, 60}, {NOTE_REST, 40},
    // Laser down-sweep (stepped)
    {1200, 35}, {1000, 35}, {800, 35}, {600, 35}, {400, 45},
    {NOTE_REST, 50},
    // Happy blips
    {NOTE_E5, 80}, {NOTE_G5, 80}, {NOTE_E6, 120},
    {NOTE_REST, 30},
    {NOTE_C6, 60}, {NOTE_G5, 60}, {NOTE_C6, 140},
  };
  playTune(tune, (int)(sizeof(tune) / sizeof(tune[0])));
}

static void buzzerTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    buzzerBusy = true;
    playFunKidSound();
    buzzerBusy = false;
  }
}

static void triggerBuzzer() {
  if (buzzerTaskHandle == nullptr) return;
  xTaskNotifyGive(buzzerTaskHandle);
}

static void initBuzzer() {
  pinMode(BUZZER_A, OUTPUT);
  pinMode(BUZZER_B, OUTPUT);
  buzzerIdlePins();
  xTaskCreatePinnedToCore(buzzerTask, "Buzzer", 3072, nullptr, 2, &buzzerTaskHandle, 0);
}

static bool pressed(uint16_t b) {
  return (buttons & b) && !(prevButtons & b);
}

static bool held(uint16_t b) {
  return buttons & b;
}

static float mapStick(uint8_t raw, bool invert) {
  int c = (int)raw - 128;
  float n = (c >= 0) ? c / 127.0f : c / 128.0f;
  if (invert) n = -n;
  if (fabsf(n) < DEADBAND) return 0.0f;
  float s = (n >= 0) ? 1.0f : -1.0f;
  float v = (fabsf(n) - DEADBAND) / (1.0f - DEADBAND);
  v = v * v * s;
  return constrain(v, -1.0f, 1.0f);
}

static void driveMecanum(float forward, float strafe, float rotate) {
  float fl = forward + strafe + rotate;
  float fr = forward - strafe - rotate;
  float rl = forward - strafe + rotate;
  float rr = forward + strafe - rotate;
  auto clamp1 = [](float& v) { if (v > 1) v = 1; if (v < -1) v = -1; };
  clamp1(fl); clamp1(fr); clamp1(rl); clamp1(rr);
  m1->setSpeed(fl);
  m2->setSpeed(fr);
  m3->setSpeed(rl);
  m4->setSpeed(rr);
}

static void stopAll() {
  if (m1) m1->stop();
  if (m2) m2->stop();
  if (m3) m3->stop();
  if (m4) m4->stop();
}

static void setArmedLed(bool on) {
  digitalWrite(LED_ARMED, on ? HIGH : LOW);
}

static void setLeftEye(bool on) {
  leftEyeOn = on;
  digitalWrite(LED_LEFT, on ? HIGH : LOW);
}

static void setRightEye(bool on) {
  rightEyeOn = on;
  digitalWrite(LED_RIGHT, on ? HIGH : LOW);
}

static void eyesBoth(bool on) {
  setLeftEye(on);
  setRightEye(on);
}

static void startWinkOnce() {
  eyeMode = EyeMode::WinkOnce;
  eyeStep = 0;
  eyeStepAt = millis();
  eyeWinkLeft = (millis() & 1);  // alternate which eye
  eyesBoth(true);
  Serial.println(eyeWinkLeft ? "Wink (left)" : "Wink (right)");
}

// ~45s playful wink / blink / stare routine
static const EyeCue EYE_SHOW[] = {
  // Wake / settle
  {800,  true,  true},
  {120,  false, false},
  {400,  true,  true},
  // Slow double-blink
  {90,   false, false},
  {220,  true,  true},
  {90,   false, false},
  {600,  true,  true},
  // Coy left wink
  {180,  false, true},
  {500,  true,  true},
  {180,  false, true},
  {900,  true,  true},
  // Coy right wink
  {180,  true,  false},
  {450,  true,  true},
  {200,  true,  false},
  {800,  true,  true},
  // Alternating chatter
  {140,  false, true},
  {140,  true,  false},
  {140,  false, true},
  {140,  true,  false},
  {140,  false, true},
  {140,  true,  false},
  {700,  true,  true},
  // Long sleepy blink
  {700,  false, false},
  {1100, true,  true},
  // Peek left
  {900,  true,  false},
  {400,  true,  true},
  // Peek right
  {900,  false, true},
  {500,  true,  true},
  // Rapid flutter
  {60,   false, false},
  {80,   true,  true},
  {60,   false, false},
  {80,   true,  true},
  {60,   false, false},
  {80,   true,  true},
  {60,   false, false},
  {500,  true,  true},
  // Dramatic one-eye hold
  {3500, false, true},
  {800,  true,  true},
  {3500, true,  false},
  {1000, true,  true},
  // Sync heartbeat blinks
  {100,  false, false},
  {280,  true,  true},
  {100,  false, false},
  {1200, true,  true},
  {100,  false, false},
  {280,  true,  true},
  {100,  false, false},
  {1500, true,  true},
  // Mischief: wink-wink
  {160,  false, true},
  {220,  true,  true},
  {160,  true,  false},
  {220,  true,  true},
  {160,  false, true},
  {220,  true,  true},
  {160,  true,  false},
  {1400, true,  true},
  // Slow cross-fade stares
  {1000, true,  false},
  {1000, false, true},
  {1000, true,  false},
  {1000, false, true},
  {1200, true,  true},
  // Final big wink + settle awake
  {120,  false, false},
  {350,  true,  true},
  {500,  false, true},
  {2800, true,  true},
  {150,  false, false},
  {1200, true,  true},
};

static const int EYE_SHOW_LEN = sizeof(EYE_SHOW) / sizeof(EYE_SHOW[0]);

static void startEyeShow() {
  eyeMode = EyeMode::Show;
  eyeStep = 0;
  eyeStepAt = millis();
  setLeftEye(EYE_SHOW[0].left);
  setRightEye(EYE_SHOW[0].right);
  Serial.println("Eye show (~45s) — press X to cut to a wink");
}

static void serviceEyes() {
  if (eyeMode == EyeMode::Idle) return;

  uint32_t now = millis();

  if (eyeMode == EyeMode::WinkOnce) {
    // both on → close one → open
    static const uint16_t phases[] = {40, 220, 180};
    if (now - eyeStepAt < phases[eyeStep]) return;
    eyeStepAt = now;
    eyeStep++;
    if (eyeStep == 1) {
      if (eyeWinkLeft) setLeftEye(false);
      else             setRightEye(false);
    } else if (eyeStep == 2) {
      eyesBoth(true);
    } else {
      eyeMode = EyeMode::Idle;
    }
    return;
  }

  // Long show
  if (eyeStep >= EYE_SHOW_LEN) {
    eyesBoth(true);
    eyeMode = EyeMode::Idle;
    Serial.println("Eye show done");
    return;
  }
  if (now - eyeStepAt < EYE_SHOW[eyeStep].ms) return;
  eyeStepAt = now;
  eyeStep++;
  if (eyeStep >= EYE_SHOW_LEN) {
    eyesBoth(true);
    eyeMode = EyeMode::Idle;
    Serial.println("Eye show done");
    return;
  }
  setLeftEye(EYE_SHOW[eyeStep].left);
  setRightEye(EYE_SHOW[eyeStep].right);
}

static void setRumble(bool on) {
  psx.setRumble(on ? 0xFF : 0x00, on ? 0x80 : 0x00);
}

static void holdOledPower() {
  gpio_hold_dis((gpio_num_t)OLED_VEXT);
  pinMode(OLED_VEXT, OUTPUT);
  digitalWrite(OLED_VEXT, LOW);
  gpio_hold_en((gpio_num_t)OLED_VEXT);
}

static bool readPs2() {
  PSX::PSXDATA js;
  if (psx.read(js) != PSXERROR_SUCCESS) {
    if (lastPs2Ok != 0 && millis() - lastPs2Ok > 100) {
      armed = false;
      estop = true;
      setRumble(false);
      setArmedLed(false);
      stopAll();
    }
    return false;
  }
  lastPs2Ok = millis();
  prevButtons = buttons;
  buttons = js.buttons;

  if (estop && held(PSXBTN_SELECT) && held(PSXBTN_START)) {
    if (estopClearStart == 0) estopClearStart = millis();
    else if (millis() - estopClearStart > 1000) {
      estop = false;
      estopClearStart = 0;
      armed = false;
      Serial.println("ESTOP cleared — press START to arm");
    }
  } else {
    estopClearStart = 0;
  }

  if (!estop && held(PSXBTN_START) && held(PSXBTN_TRIANGLE)) {
    estop = true;
    armed = false;
    setRumble(false);
    setArmedLed(false);
    stopAll();
    Serial.println("ESTOP — hold SELECT+START to clear");
  }

  if (!estop) {
    if (held(PSXBTN_L1))      speedScale = 0.35f;
    else if (held(PSXBTN_R1)) speedScale = 1.00f;
    else if (held(PSXBTN_L2)) speedScale = 0.50f;
    else                       speedScale = 0.70f;
  }

  // Full sticks for RGB mode (Y inverted so up = positive)
  stickLX = mapStick(js.JoyLeftX, false);
  stickLY = mapStick(js.JoyLeftY, true);
  stickRX = mapStick(js.JoyRightX, false);
  stickRY = mapStick(js.JoyRightY, true);

  vx = stickLY;
  vy = stickRX;
  wz = stickLX;
  if (held(PSXBTN_UP))    vx = 1.0f;
  if (held(PSXBTN_DOWN))  vx = -1.0f;
  if (held(PSXBTN_LEFT))  wz = -1.0f;
  if (held(PSXBTN_RIGHT)) wz = 1.0f;

  return true;
}

static void initOledOnce() {
  holdOledPower();
  delay(100);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
  gpio_hold_en((gpio_num_t)OLED_RST);

  // Paint once — no further I2C (motor EMI corrupts transfers)
  SSD1306Wire display(OLED_ADDR, OLED_SDA, OLED_SCL, GEOMETRY_128_64, I2C_TWO, 100000);
  display.init();
  display.flipScreenVertically();
  display.setContrast(255);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 24, "Hello Adei");
  display.display();
  Serial.println("OLED ready");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Mecanum open-loop");
  Serial.println("  START = arm/disarm");
  Serial.println("  START+TRIANGLE = ESTOP");
  Serial.println("  SELECT = RGB LED mode (joystick fun)");
  Serial.println("  SQUARE/CIRCLE = toggle eyes (or RGB effects in LED mode)");
  Serial.println("  X = wink once   TRIANGLE = eye show (~45s)");
  Serial.println("  R2 = fun buzzer");

  pinMode(LED_ARMED, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  setArmedLed(false);
  setLeftEye(false);
  setRightEye(false);
  initBuzzer();

  initOledOnce();

  m1 = new MotorDriver(0, 1, M1_RPWM, M1_LPWM);
  m2 = new MotorDriver(2, 3, M2_RPWM, M2_LPWM);
  m3 = new MotorDriver(4, 5, M3_RPWM, M3_LPWM);
  m4 = new MotorDriver(6, 7, M4_RPWM, M4_LPWM);
  holdOledPower();  // re-lock after LEDC attach

  if (!rgbLed.begin()) {
    Serial.println("RGB LED init failed (check GPIO 33 wiring)");
  } else {
    Serial.println("RGB LED ready on GPIO 33 (7x WS2812)");
  }

  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
  psx.config(PSXMODE_ANALOG);
  psx.setRumble(0, 0);
  delay(300);

  stopAll();
  Serial.println("Ready (DISARMED)");
}

void loop() {
  bool ok = readPs2();

  if (estop || !ok) {
    stopAll();
    setRumble(false);
    if (ledMode) {
      ledMode = false;
      rgbLed.setEnabled(false);
    }
    holdOledPower();
    delay(20);
    return;
  }

  if (pressed(PSXBTN_START) && !held(PSXBTN_SELECT) && !held(PSXBTN_TRIANGLE)) {
    armed = !armed;
    setArmedLed(armed);
    setRumble(false);
    if (!armed) stopAll();
    Serial.println(armed ? "ARMED" : "DISARMED");
  }

  // SELECT alone toggles RGB playground (SELECT+START is ESTOP clear)
  if (pressed(PSXBTN_SELECT) && !held(PSXBTN_START)) {
    ledMode = !ledMode;
    rgbLed.setEnabled(ledMode);
    stopAll();
    setRumble(false);
    if (ledMode) {
      Serial.print("LED mode ON — effect: ");
      Serial.println(rgbLed.effectName());
      Serial.println("  SQUARE/CIRCLE = prev/next effect");
      Serial.println("  sticks drive the active effect");
    } else {
      Serial.println("LED mode OFF — drive restored");
    }
  }

  if (ledMode) {
    if (pressed(PSXBTN_SQUARE)) {
      rgbLed.prevEffect();
      Serial.print("RGB effect: ");
      Serial.println(rgbLed.effectName());
    }
    if (pressed(PSXBTN_CIRCLE)) {
      rgbLed.nextEffect();
      Serial.print("RGB effect: ");
      Serial.println(rgbLed.effectName());
    }
    if (pressed(PSXBTN_CROSS)) {
      rgbLed.setEffect(RgbEffect::Comet);
      Serial.println("RGB effect: Comet");
    }
    if (pressed(PSXBTN_TRIANGLE) && !held(PSXBTN_START)) {
      rgbLed.setEffect(RgbEffect::RainbowSpin);
      Serial.println("RGB effect: RainbowSpin");
    }
    if (pressed(PSXBTN_R2)) {
      triggerBuzzer();
    }

    stopAll();
    setRumble(false);
    rgbLed.update(stickLX, stickLY, stickRX, stickRY);
  } else {
    if (pressed(PSXBTN_SQUARE)) {
      setLeftEye(!leftEyeOn);
    }
    if (pressed(PSXBTN_CIRCLE)) {
      setRightEye(!rightEyeOn);
    }

    // X = single wink; Triangle alone = long show (START+TRIANGLE is ESTOP)
    if (pressed(PSXBTN_CROSS)) {
      startWinkOnce();
    }
    if (pressed(PSXBTN_TRIANGLE) && !held(PSXBTN_START)) {
      startEyeShow();
    }
    if (pressed(PSXBTN_R2)) {
      triggerBuzzer();
    }

    serviceEyes();

    const bool motorInput = fabsf(vx) > 0.02f || fabsf(vy) > 0.02f || fabsf(wz) > 0.02f;

    if (!armed) {
      stopAll();
      setRumble(motorInput);
    } else {
      setRumble(false);
      driveMecanum(vx * speedScale, vy * speedScale, wz * speedScale);
    }
  }

  holdOledPower();
  delay(20);
}
