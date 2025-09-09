#include <Arduino.h>
#include <PSX.h>

// ---- Robot Physical Parameters ----
#define WHEELBASE_INCHES 10.75f    // Distance between wheels (inches)
#define WHEEL_DIAMETER_MM 80       // Wheel diameter (mm)
#define WHEELBASE_METERS (WHEELBASE_INCHES * 0.0254f)  // Convert to meters

// ---- PS2 pins (working) ----
#define PIN_PS2_ATT  5
#define PIN_PS2_CLK  7
#define PIN_PS2_CMD  6
#define PIN_PS2_DAT  4

// ---- BTS7960 pins (dual-PWM) ----
#define M1_RPWM 46   // Front Left motor
#define M1_LPWM 1
#define M2_RPWM 2    // Front Right motor
#define M2_LPWM 3
#define M3_RPWM 36   // Rear Left motor
#define M3_LPWM 37
#define M4_RPWM 45   // Rear Right motor
#define M4_LPWM 19

// ---- LEDC channels ----
enum {
  CH_M1_R, CH_M1_L,  // Front Left
  CH_M2_R, CH_M2_L,  // Front Right
  CH_M3_R, CH_M3_L,  // Rear Left
  CH_M4_R, CH_M4_L   // Rear Right
};

// ---- PWM Configuration - Optimized for TT Motors ----
static const int PWM_FREQ = 16000;  // 16 kHz - better for small TT motors
static const int PWM_RES  = 10;     // 10-bit (0–1023)
static const int PWM_MAX  = (1 << PWM_RES) - 1;

// ---- Control Parameters ----
static const float WHEELBASE_HALF = WHEELBASE_METERS / 2.0f;  // Corrected geometry
static const float DEADBAND = 0.12f;        // Larger deadband for better control
static const float SPEED_SMOOTH = 0.85f;    // Speed smoothing factor

// ---- Globals ----
PSX psx;
float speed_scale = 0.70f;  // Start with more conservative speed
bool controller_connected = false;
uint32_t last_controller_read = 0;

// Motor speed smoothing
float motor_speeds[4] = {0, 0, 0, 0};
float target_speeds[4] = {0, 0, 0, 0};

// ---- Enhanced PWM Setup ----
void setupPWM() {
  Serial.println("Setting up PWM channels...");
  
  // Setup all PWM channels with optimized settings for TT motors
  ledcSetup(CH_M1_R, PWM_FREQ, PWM_RES); ledcAttachPin(M1_RPWM, CH_M1_R);
  ledcSetup(CH_M1_L, PWM_FREQ, PWM_RES); ledcAttachPin(M1_LPWM, CH_M1_L);
  ledcSetup(CH_M2_R, PWM_FREQ, PWM_RES); ledcAttachPin(M2_RPWM, CH_M2_R);
  ledcSetup(CH_M2_L, PWM_FREQ, PWM_RES); ledcAttachPin(M2_LPWM, CH_M2_L);
  ledcSetup(CH_M3_R, PWM_FREQ, PWM_RES); ledcAttachPin(M3_RPWM, CH_M3_R);
  ledcSetup(CH_M3_L, PWM_FREQ, PWM_RES); ledcAttachPin(M3_LPWM, CH_M3_L);
  ledcSetup(CH_M4_R, PWM_FREQ, PWM_RES); ledcAttachPin(M4_RPWM, CH_M4_R);
  ledcSetup(CH_M4_L, PWM_FREQ, PWM_RES); ledcAttachPin(M4_LPWM, CH_M4_L);
  
  Serial.println("PWM setup complete");
}

// ---- Enhanced Motor Driver with Safety ----
void driveBTS7960(int chR, int chL, int duty, int dir) {
  // Clamp duty cycle
  duty = constrain(duty, 0, PWM_MAX);
  
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

// ---- Emergency Stop ----
void emergencyStop() {
  driveBTS7960(CH_M1_R, CH_M1_L, 0, 0);
  driveBTS7960(CH_M2_R, CH_M2_L, 0, 0);
  driveBTS7960(CH_M3_R, CH_M3_L, 0, 0);
  driveBTS7960(CH_M4_R, CH_M4_L, 0, 0);
  
  // Reset motor speeds
  for (int i = 0; i < 4; i++) {
    motor_speeds[i] = 0;
    target_speeds[i] = 0;
  }
}

// ---- Enhanced Stick Mapping with Better Feel ----
static inline float mapStick(uint8_t rawValue, bool invert = false) {
  // Convert 0-255 to -128 to +127
  int centered = int(rawValue) - 128;
  
  // Normalize to -1.0 to +1.0
  float normalized = (centered >= 0) ? centered / 127.0f : centered / 128.0f;
  
  // Apply inversion if requested
  if (invert) normalized = -normalized;
  
  // Apply deadband
  if (fabsf(normalized) < DEADBAND) return 0.0f;
  
  // Scale remaining range to maintain full -1 to +1 output
  float sign = (normalized >= 0) ? 1.0f : -1.0f;
  float scaled = (fabsf(normalized) - DEADBAND) / (1.0f - DEADBAND);
  
  // Apply slight exponential curve for finer control near center
  scaled = scaled * scaled * sign;
  
  return constrain(scaled, -1.0f, 1.0f);
}

// ---- Setup ----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("Mecanum Robot Controller v2.0");
  Serial.println("Wheelbase: " + String(WHEELBASE_INCHES) + "\" square");
  Serial.println("Wheel Diameter: " + String(WHEEL_DIAMETER_MM) + "mm");
  Serial.println("========================================");

  setupPWM();
  
  // Initialize PS2 controller
  Serial.println("Initializing PS2 controller...");
  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
  psx.config(PSXMODE_ANALOG);
  
  delay(500);
  Serial.println("Setup complete. Waiting for controller...");
}

// ---- Main Control Loop ----
void loop() {
  PSX::PSXDATA js;
  uint32_t now = millis();
  
  // Try to read controller
  if (psx.read(js) == PSXERROR_SUCCESS) {
    controller_connected = true;
    last_controller_read = now;
  } else {
    // Controller timeout handling
    if (now - last_controller_read > 100) {  // 100ms timeout
      if (controller_connected) {
        Serial.println("Controller disconnected!");
        controller_connected = false;
      }
      emergencyStop();
      delay(10);
      return;
    }
  }
  
  if (!controller_connected) {
    delay(10);
    return;
  }
  
  // ---- Speed Mode Control ----
  if (js.buttons & PSXBTN_L1) {
    speed_scale = 0.35f;  // Slow mode for precision
  } else if (js.buttons & PSXBTN_R1) {
    speed_scale = 1.00f;  // Full speed mode
  } else if (js.buttons & PSXBTN_L2) {
    speed_scale = 0.50f;  // Medium-slow mode
  } else {
    speed_scale = 0.70f;  // Default mode
  }
  
  // ---- Emergency Stop ----
  if (js.buttons & PSXBTN_SELECT) {
    emergencyStop();
    Serial.println("EMERGENCY STOP!");
    delay(100);
    return;
  }
  
  // ---- Get Joystick Commands ----
  float vx = mapStick(js.JoyLeftY, true);   // Forward/backward (inverted)
  float vy = mapStick(js.JoyLeftX, false);  // Left/right strafe
  float wz = mapStick(js.JoyRightX, false); // Rotation (right stick X)
  
  // ---- Mecanum Wheel Kinematics ----
  // Using corrected geometry for 10.75" square wheelbase
  float front_left  =  vx - vy - WHEELBASE_HALF * wz;
  float front_right =  vx + vy + WHEELBASE_HALF * wz;
  float rear_left   =  vx + vy - WHEELBASE_HALF * wz;
  float rear_right  =  vx - vy + WHEELBASE_HALF * wz;
  
  // ---- Normalize to prevent saturation ----
  float max_speed = fmaxf(fmaxf(fabsf(front_left), fabsf(front_right)), 
                         fmaxf(fabsf(rear_left), fabsf(rear_right)));
  
  if (max_speed > 1.0f) {
    front_left  /= max_speed;
    front_right /= max_speed;
    rear_left   /= max_speed;
    rear_right  /= max_speed;
  }
  
  // ---- Apply Speed Scaling ----
  target_speeds[0] = front_left  * speed_scale;
  target_speeds[1] = front_right * speed_scale;
  target_speeds[2] = rear_left   * speed_scale;
  target_speeds[3] = rear_right  * speed_scale;
  
  // ---- Motor Direction Corrections (adjust as needed) ----
  // You mentioned M4 was reversed, keeping that fix
  target_speeds[3] = -target_speeds[3];  // Rear Right reversed
  
  // ---- Smooth Motor Speed Changes ----
  for (int i = 0; i < 4; i++) {
    motor_speeds[i] = motor_speeds[i] * SPEED_SMOOTH + target_speeds[i] * (1.0f - SPEED_SMOOTH);
  }
  
  // ---- Drive Motors ----
  auto toDuty = [](float speed) { return int(fabsf(speed) * PWM_MAX); };
  auto getDirection = [](float speed) { return (speed > 0) ? 1 : (speed < 0) ? -1 : 0; };
  
  driveBTS7960(CH_M1_R, CH_M1_L, toDuty(motor_speeds[0]), getDirection(motor_speeds[0]));
  driveBTS7960(CH_M2_R, CH_M2_L, toDuty(motor_speeds[1]), getDirection(motor_speeds[1]));
  driveBTS7960(CH_M3_R, CH_M3_L, toDuty(motor_speeds[2]), getDirection(motor_speeds[2]));
  driveBTS7960(CH_M4_R, CH_M4_L, toDuty(motor_speeds[3]), getDirection(motor_speeds[3]));
  
  // ---- Status Reporting ----
  static uint32_t last_status = 0;
  if (now - last_status > 500) {  // Every 500ms
    Serial.printf("vx=%.2f vy=%.2f wz=%.2f | speed=%.2f | FL=%.2f FR=%.2f RL=%.2f RR=%.2f\n",
                  vx, vy, wz, speed_scale, 
                  motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
    last_status = now;
  }
  
  delay(10);  // 100Hz control loop
}
