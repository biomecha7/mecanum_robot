#include <Arduino.h>
#include <PSX.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "EncoderDriverSimple.h"
#include "MotorDriver.h"
#include "MotionController.h"

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

// Encoder pins
#define ENC_M1_A  21  // Front Left encoder A
#define ENC_M1_B  20  // Front Left encoder B
#define ENC_M2_A  26  // Front Right encoder A  
#define ENC_M2_B  48  // Front Right encoder B
#define ENC_M3_A  47  // Rear Left encoder A
#define ENC_M3_B  33  // Rear Left encoder B
#define ENC_M4_A  34  // Rear Right encoder A
#define ENC_M4_B  35  // Rear Right encoder B

// IMU pins
#define IMU_SDA  38
#define IMU_SCL  39

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
static const float ROTATION_MULTIPLIER = 4.5f;  // Increased rotation sensitivity
static const float DEADBAND = 0.10f;        // Slightly smaller deadband for more responsive control
static const float SPEED_SMOOTH = 0.80f;    // Less smoothing for more responsive feel

// ---- Globals ----
PSX psx;
float speed_scale = 0.70f;  // Start with more conservative speed

// Encoder counts
volatile int32_t encoder_counts[4] = {0, 0, 0, 0};

// Encoder driver objects
EncoderDriverSimple encoder1(ENC_M1_A, ENC_M1_B, &encoder_counts[0]);
EncoderDriverSimple encoder2(ENC_M2_A, ENC_M2_B, &encoder_counts[1]);
EncoderDriverSimple encoder3(ENC_M3_A, ENC_M3_B, &encoder_counts[2]);
EncoderDriverSimple encoder4(ENC_M4_A, ENC_M4_B, &encoder_counts[3]);

// Motor driver objects
MotorDriver motorFL(CH_M1_R, CH_M1_L, M1_RPWM, M1_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motorFR(CH_M2_R, CH_M2_L, M2_RPWM, M2_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motorRL(CH_M3_R, CH_M3_L, M3_RPWM, M3_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motorRR(CH_M4_R, CH_M4_L, M4_RPWM, M4_LPWM, PWM_FREQ, PWM_RES);

// Motion controller
MotionController motionController(&motorFL, &motorFR, &motorRL, &motorRR);

// IMU
ICM_20948_I2C myICM;
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

void setupEncoders() {
  Serial.println("🔧 Setting up encoders...");
  encoder1.begin();
  delay(100);
  encoder2.begin();
  delay(100);
  encoder3.begin();
  delay(100);
  encoder4.begin();
  delay(100);
  Serial.println("✅ Encoders setup complete");
}

void setupI2C() {
  Serial.println("🔧 Setting up I2C...");
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  Serial.println("✅ I2C setup complete");
}

void initIMU() {
  Serial.println("🔧 Initializing IMU...");
  if (myICM.begin(Wire, 0) == ICM_20948_Stat_Ok) {
    Serial.println("✅ IMU initialized at address 0x68");
  } else if (myICM.begin(Wire, 1) == ICM_20948_Stat_Ok) {
    Serial.println("✅ IMU initialized at address 0x69");
  } else {
    Serial.println("❌ IMU initialization failed");
  }
  Serial.println("✅ IMU setup complete");
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
  motionController.stop();
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
  Serial.println("🤖 Mecanum Robot Controller - DRIVE MODE");
  Serial.println("Wheelbase: " + String(WHEELBASE_INCHES) + "\" square");
  Serial.println("Wheel Diameter: " + String(WHEEL_DIAMETER_MM) + "mm");
  Serial.println("========================================");
  Serial.println("🎮 CONTROLS:");
  Serial.println("  X: Drive (hold to move)");
  Serial.println("  L1: Slow mode (35%)");
  Serial.println("  R1: Fast mode (100%)");
  Serial.println("  L2: Medium mode (50%)");
  Serial.println("  SELECT: Emergency Stop");
  Serial.println("  R2: Show encoder values");
  Serial.println("========================================");

  setupPWM();
  setupEncoders();
  setupI2C();
  initIMU();
  
  // Initialize PS2 controller
  Serial.println("Initializing PS2 controller...");
  psx.setupPins(PIN_PS2_DAT, PIN_PS2_CMD, PIN_PS2_ATT, PIN_PS2_CLK, 10);
  psx.config(PSXMODE_ANALOG);
  
  delay(500);
  Serial.println("Setup complete. Ready to drive!");
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
  
  // ---- Emergency Stop ----
  if (js.buttons & PSXBTN_SELECT) {
    emergencyStop();
    Serial.println("🛑 EMERGENCY STOP!");
    delay(100);
    return;
  }
  
  // ---- Show Encoder Values ----
  if (js.buttons & PSXBTN_R2) {
    static uint32_t last_encoder_print = 0;
    if (now - last_encoder_print > 200) {
      Serial.printf("📊 ENCODERS: M1=%d M2=%d M3=%d M4=%d\n",
                    encoder_counts[0], encoder_counts[1], 
                    encoder_counts[2], encoder_counts[3]);
      last_encoder_print = now;
    }
  }
  
  // ---- Drive Mode ----
  // Speed Mode Control
  if (js.buttons & PSXBTN_L1) {
    speed_scale = 0.35f;  // Slow mode for precision
  } else if (js.buttons & PSXBTN_R1) {
    speed_scale = 1.00f;  // Full speed mode
  } else if (js.buttons & PSXBTN_L2) {
    speed_scale = 0.50f;  // Medium-slow mode
  } else {
    speed_scale = 0.70f;  // Default mode
  }
  
  // ---- Get Joystick Commands ----
  float vx = mapStick(js.JoyLeftY, true);   // Forward/backward (inverted)
  float vy = mapStick(js.JoyRightX, false); // Left/right strafe (swapped)
  float wz = mapStick(js.JoyLeftX, false);  // Rotation (swapped)

  // ---- Drive using MotionController ----
  motionController.drive(vx * speed_scale, vy * speed_scale, wz * speed_scale);

  // ---- Status Reporting ----
  static uint32_t last_status = 0;
  if (now - last_status > 500) {  // Every 500ms
    Serial.printf("vx=%.2f vy=%.2f wz=%.2f | speed=%.2f\n",
                  vx, vy, wz, speed_scale);
    last_status = now;
  }

  
  delay(10);  // 100Hz control loop
}
