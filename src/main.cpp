#include <Arduino.h>
#include <PSX.h>

// ---- PS2 pins (working) ----
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7
#define PIN_PS2_CMD  6
#define PIN_PS2_DAT  4

// ---- BTS7960 pins (dual-PWM) ----
#define M1_RPWM 46
#define M1_LPWM 1
#define M2_RPWM 2
#define M2_LPWM 3
#define M3_RPWM 36
#define M3_LPWM 37
#define M4_RPWM 45
#define M4_LPWM 19   // revised, no GPIO16 on Heltec V3

// ---- LEDC channels ----
enum {
  CH_M1_R, CH_M1_L,
  CH_M2_R, CH_M2_L,
  CH_M3_R, CH_M3_L,
  CH_M4_R, CH_M4_L
};

static const int PWM_FREQ = 20000; // 20 kHz
static const int PWM_RES  = 10;    // 10-bit (0–1023)

// ---- Globals ----
PSX psx;
PSX::PSXDATA state;

// ---- Helpers ----
void setupPWM() {
  ledcSetup(CH_M1_R, PWM_FREQ, PWM_RES); ledcAttachPin(M1_RPWM, CH_M1_R);
  ledcSetup(CH_M1_L, PWM_FREQ, PWM_RES); ledcAttachPin(M1_LPWM, CH_M1_L);
  ledcSetup(CH_M2_R, PWM_FREQ, PWM_RES); ledcAttachPin(M2_RPWM, CH_M2_R);
  ledcSetup(CH_M2_L, PWM_FREQ, PWM_RES); ledcAttachPin(M2_LPWM, CH_M2_L);
  ledcSetup(CH_M3_R, PWM_FREQ, PWM_RES); ledcAttachPin(M3_RPWM, CH_M3_R);
  ledcSetup(CH_M3_L, PWM_FREQ, PWM_RES); ledcAttachPin(M3_LPWM, CH_M3_L);
  ledcSetup(CH_M4_R, PWM_FREQ, PWM_RES); ledcAttachPin(M4_RPWM, CH_M4_R);
  ledcSetup(CH_M4_L, PWM_FREQ, PWM_RES); ledcAttachPin(M4_LPWM, CH_M4_L);
}

void driveBTS7960(int chR, int chL, int duty, int dir) {
  // dir=+1 forward, -1 reverse, 0 stop
  if (dir > 0) {
    ledcWrite(chR, duty);
    ledcWrite(chL, 0);
  } else if (dir < 0) {
    ledcWrite(chR, 0);
    ledcWrite(chL, duty);
  } else {
    ledcWrite(chR, 0);
    ledcWrite(chL, 0);
  }
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[Robot] PS2 motor test");

  setupPWM();

  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
  psx.config(PSXMODE_ANALOG);
}

// ---- tuning ----
static const int PWM_RES  = 10;      // already used in your setup
static const float k_geom = 0.27305f; // (L+W)/2 in meters for 10.75" square
float speed_scale = 0.85f;            // default max

// map PS2 stick (0..255) -> -1..+1 with deadband
static inline float mapStick(uint8_t v, bool invert=false) {
  int c = int(v) - 128;
  float x = (c >= 0) ? c / 127.0f : c / 128.0f;
  if (invert) x = -x;
  const float dead = 0.08f;
  if (fabsf(x) < dead) return 0.0f;
  float s = (fabsf(x) - dead) / (1.0f - dead);
  return (x >= 0) ? s : -s;
}

void loop() {
  PSX::PSXDATA js;
  if (psx.read(js) != PSXERROR_SUCCESS) {
    // fail-safe stop if controller not readable
    driveBTS7960(CH_M1_R, CH_M1_L, 0, 0);
    driveBTS7960(CH_M2_R, CH_M2_L, 0, 0);
    driveBTS7960(CH_M3_R, CH_M3_L, 0, 0);
    driveBTS7960(CH_M4_R, CH_M4_L, 0, 0);
    delay(10);
    return;
  }

  // speed modes
  if (js.buttons & PSXBTN_L1)      speed_scale = 0.45f;
  else if (js.buttons & PSXBTN_R1) speed_scale = 1.00f;
  else                              speed_scale = 0.85f;

  // sticks -> commands
  float vx = mapStick(js.JoyLeftY,  true);  // up=forward
  float vy = mapStick(js.JoyLeftX,  false); // right=strafe right
  float wz = mapStick(js.JoyRightX, false); // right=rotate CW (flip if you prefer)

  // mecanum mix (FL, FR, RL, RR)
  float FL =  vx - vy - k_geom*wz;
  float FR =  vx + vy + k_geom*wz;
  float RL =  vx + vy - k_geom*wz;
  float RR =  vx - vy + k_geom*wz;

  // normalize to [-1,1]
  float m = fmaxf(fmaxf(fabsf(FL), fabsf(FR)), fmaxf(fabsf(RL), fabsf(RR)));
  if (m > 1.0f) { FL/=m; FR/=m; RL/=m; RR/=m; }

  // global scaling
  FL *= speed_scale; FR *= speed_scale; RL *= speed_scale; RR *= speed_scale;

  // --- software fix for your reversed M4 ---
  RR = -RR;

  // send to BTS7960 (dual-PWM helper you already have)
  auto toDuty = [](float v){ return int(fabsf(v) * ((1<<PWM_RES)-1)); };
  driveBTS7960(CH_M1_R, CH_M1_L, toDuty(FL), (FL>0)-(FL<0));
  driveBTS7960(CH_M2_R, CH_M2_L, toDuty(FR), (FR>0)-(FR<0));
  driveBTS7960(CH_M3_R, CH_M3_L, toDuty(RL), (RL>0)-(RL<0));
  driveBTS7960(CH_M4_R, CH_M4_L, toDuty(RR), (RR>0)-(RR<0));

  // optional: brief status log
  static uint32_t t=0; if (millis()-t>250) {
    Serial.printf("vx=%.2f vy=%.2f wz=%.2f  scale=%.2f  RRfix\n",
                  vx, vy, wz, speed_scale);
    t = millis();
  }
  delay(10);
}
