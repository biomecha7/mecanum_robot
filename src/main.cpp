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

// ---- Loop ----
void loop() {
  int err = psx.read(state);
  if (err != PSXERROR_SUCCESS) {
    static uint32_t tErr=0; if (millis()-tErr > 1000) {
      Serial.println("[PS2] No data…");
      tErr = millis();
    }
    delay(10);
    return;
  }

  const int duty = 700; // ~70% of 10-bit max = ~1023
  int dirM1=0, dirM2=0, dirM3=0, dirM4=0;

  // --- Map buttons to motor directions ---
  if (state.buttons & PSXBTN_UP)    dirM1 = +1;
  if (state.buttons & PSXBTN_DOWN)  dirM1 = -1;

  if (state.buttons & PSXBTN_RIGHT) dirM2 = +1;
  if (state.buttons & PSXBTN_LEFT)  dirM2 = -1;

  if (state.buttons & PSXBTN_TRIANGLE) dirM3 = +1;
  if (state.buttons & PSXBTN_CROSS)    dirM3 = -1;

  if (state.buttons & PSXBTN_SQUARE) dirM4 = +1;
  if (state.buttons & PSXBTN_CIRCLE) dirM4 = -1;

  // --- Drive motors ---
  driveBTS7960(CH_M1_R, CH_M1_L, duty, dirM1);
  driveBTS7960(CH_M2_R, CH_M2_L, duty, dirM2);
  driveBTS7960(CH_M3_R, CH_M3_L, duty, dirM3);
  driveBTS7960(CH_M4_R, CH_M4_L, duty, dirM4);

  delay(20);
}
