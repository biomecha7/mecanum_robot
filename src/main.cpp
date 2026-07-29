#include <Arduino.h>
#include <math.h>
#include <SSD1306Wire.h>
#include <driver/gpio.h>
#include <PSX.h>
#include "RobotPins.h"
#include "MotorDriver.h"

static MotorDriver* m1 = nullptr;
static MotorDriver* m2 = nullptr;
static MotorDriver* m3 = nullptr;
static MotorDriver* m4 = nullptr;

PSX psx;
static uint16_t buttons = 0, prevButtons = 0;
static float vx = 0, vy = 0, wz = 0;
static float speedScale = 0.70f;
static bool estop = false;
static bool armed = false;
static uint32_t estopClearStart = 0;
static uint32_t lastPs2Ok = 0;

static const float DEADBAND = 0.10f;

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

  vx = mapStick(js.JoyLeftY, true);
  vy = mapStick(js.JoyRightX, false);
  wz = mapStick(js.JoyLeftX, false);
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

  pinMode(LED_ARMED, OUTPUT);
  setArmedLed(false);

  initOledOnce();

  m1 = new MotorDriver(0, 1, M1_RPWM, M1_LPWM);
  m2 = new MotorDriver(2, 3, M2_RPWM, M2_LPWM);
  m3 = new MotorDriver(4, 5, M3_RPWM, M3_LPWM);
  m4 = new MotorDriver(6, 7, M4_RPWM, M4_LPWM);
  holdOledPower();  // re-lock after LEDC attach

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

  const bool motorInput = fabsf(vx) > 0.02f || fabsf(vy) > 0.02f || fabsf(wz) > 0.02f;

  if (!armed) {
    stopAll();
    setRumble(motorInput);
  } else {
    setRumble(false);
    driveMecanum(vx * speedScale, vy * speedScale, wz * speedScale);
  }

  holdOledPower();
  delay(20);
}
