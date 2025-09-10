#include <Arduino.h>
#include <PSX.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "EncoderDriverSimple.h"
#include "MotorDriver.h"
#include "IMUDriver.h"
#include "AHRS.h"

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

// IMU
IMUDriver imu(IMU_SDA, IMU_SCL);
AHRS ahrs(100.0f, 0.1f); // 100Hz, beta=0.1
bool controller_connected = false;
uint32_t last_controller_read = 0;

// Motor speed smoothing
float motor_speeds[4] = {0, 0, 0, 0};
float target_speeds[4] = {0, 0, 0, 0};

// Motor driver objects
MotorDriver motor1(CH_M1_R, CH_M1_L, M1_RPWM, M1_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motor2(CH_M2_R, CH_M2_L, M2_RPWM, M2_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motor3(CH_M3_R, CH_M3_L, M3_RPWM, M3_LPWM, PWM_FREQ, PWM_RES);
MotorDriver motor4(CH_M4_R, CH_M4_L, M4_RPWM, M4_LPWM, PWM_FREQ, PWM_RES);

// Test modes
bool encoder_test_mode = false;
bool imu_test_mode = false;
bool motor_test_mode = false;
uint32_t test_start_time = 0;

// Motor test variables (moved to global scope to avoid static issues)
int current_test_motor = 0;
uint32_t motor_test_start_time = 0;

// ---- Enhanced PWM Setup ----
void setupPWM() {
  Serial.println("Setting up MotorDriver objects...");
  // MotorDriver constructors already setup PWM
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
  // IMUDriver handles I2C setup
  Serial.println("✅ I2C setup complete");
}

// ---- Enhanced Motor Driver with Safety ----
// MotorDriver class now handles motor control

// ---- Emergency Stop ----
void emergencyStop() {
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  // Reset motor speeds
  for (int i = 0; i < 4; i++) {
    motor_speeds[i] = 0;
    target_speeds[i] = 0;
  }
}

// ---- Motor Test Function (Fixed) ----
void runMotorTest() {
  uint32_t now = millis();
  const int TEST_DURATION = 2000;  // 2 seconds per motor
  if (now - motor_test_start_time > TEST_DURATION) {
    // Stop all motors
    motor1.stop();
    motor2.stop();
    motor3.stop();
    motor4.stop();
    // Move to next motor
    current_test_motor++;
    motor_test_start_time = now;
    if (current_test_motor >= 4) {
      motor_test_mode = false;
      current_test_motor = 0;  // Reset for next time
      Serial.println("\n=== MOTOR TEST COMPLETE ===");
      return;
    }
  }
  // Test current motor
  const char* motor_names[] = {"Front Left", "Front Right", "Rear Left", "Rear Right"};
  MotorDriver* motors[] = {&motor1, &motor2, &motor3, &motor4};
  if (now - motor_test_start_time < 1000) {
    // First second: forward
    Serial.printf("Testing %s - FORWARD\n", motor_names[current_test_motor]);
    motors[current_test_motor]->drive(512, 1);
  } else {
    // Second second: backward
    Serial.printf("Testing %s - BACKWARD\n", motor_names[current_test_motor]);
    motors[current_test_motor]->drive(512, -1);
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
  Serial.println("🤖 Mecanum Robot Controller v3.1 - FIXED");
  Serial.println("Wheelbase: " + String(WHEELBASE_INCHES) + "\" square");
  Serial.println("Wheel Diameter: " + String(WHEEL_DIAMETER_MM) + "mm");
  Serial.println("========================================");
  Serial.println("🎮 CONTROLS:");
  Serial.println("  L1: Motor Test");
  Serial.println("  L2: IMU Test");
  Serial.println("  R1: Encoder Test");
  Serial.println("  R2: All Sensors Test");
  Serial.println("  START: I2C Scanner");
  Serial.println("  SELECT: Emergency Stop");
  Serial.println("  X: Normal Robot Control");
  Serial.println("========================================");

  setupPWM();
  setupEncoders();
  setupI2C();
  if (!imu.begin()) {
    Serial.println("❌ IMU initialization failed");
  } else {
    Serial.println("✅ IMU initialized");
  }
  
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
  
  // ---- Test Mode Controls ----
  if (js.buttons & PSXBTN_L1) {
    if (!motor_test_mode) {
      motor_test_mode = true;
      motor_test_start_time = millis();
      current_test_motor = 0;  // Reset motor index
      Serial.println("\n🚀 Starting Motor Test...");
    }
  } else {
    motor_test_mode = false;
  }
  
  if (js.buttons & PSXBTN_L2) {
    if (!imu_test_mode) {
      imu_test_mode = true;
      Serial.println("\n🚀 Starting IMU Test...");
    }
  } else {
    imu_test_mode = false;
  }
  
  if (js.buttons & PSXBTN_R1) {
    if (!encoder_test_mode) {
      encoder_test_mode = true;
      Serial.println("\n🚀 Starting Encoder Test...");
    }
  } else {
    encoder_test_mode = false;
  }
  
  if (js.buttons & PSXBTN_R2) {
    // All sensors test
    static uint32_t last_all_sensors = 0;
    if (now - last_all_sensors > 500) {
      Serial.println("\n🔍 === ALL SENSORS TEST ===");
      imu.update();
      const IMUData& data = imu.getData();
      Serial.printf("   IMU: Accel(%.2f,%.2f,%.2f)g | Gyro(%.1f,%.1f,%.1f)dps | Mag(%.0f,%.0f,%.0f)µT | Temp=%.1f°C\n",
                    data.accel_x, data.accel_y, data.accel_z,
                    data.gyro_x, data.gyro_y, data.gyro_z,
                    data.mag_x, data.mag_y, data.mag_z,
                    data.temperature);
      ahrs.update(data);
      const AHRSData& ahrsData = ahrs.getData();
      if (ahrsData.valid) {
        Serial.printf("   AHRS: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", ahrsData.roll, ahrsData.pitch, ahrsData.yaw);
      } else {
        Serial.println("   AHRS: Orientation not valid");
      }
      // Encoder data
      Serial.printf("   ENCODERS: M1=%d M2=%d M3=%d M4=%d\n",
                    encoder_counts[0], encoder_counts[1], 
                    encoder_counts[2], encoder_counts[3]);
      // PS2 data
      Serial.printf("   PS2: LY=%d LX=%d RY=%d RX=%d\n",
                    js.JoyLeftY, js.JoyLeftX, js.JoyRightY, js.JoyRightX);
      Serial.println("   ---");
      last_all_sensors = now;
    }
  }
  
  if (js.buttons & PSXBTN_START) {
    // I2C Scanner
    static uint32_t last_i2c_scan = 0;
    if (now - last_i2c_scan > 1000) {
      Serial.println("\n🔍 Scanning I2C devices...");
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.printf("   ✅ I2C device found at address 0x%02X\n", addr);
        }
      }
      Serial.println("   📊 I2C scan complete");
      last_i2c_scan = now;
    }
  }
  
  // ---- Emergency Stop ----
  if (js.buttons & PSXBTN_SELECT) {
    emergencyStop();
    Serial.println("🛑 EMERGENCY STOP!");
    delay(100);
    return;
  }
  
  // Run test modes
  if (motor_test_mode) {
    runMotorTest();
    return;
  }
  
  if (imu_test_mode) {
    static uint32_t last_imu_print = 0;
    if (now - last_imu_print > 200) {
      imu.update();
      const IMUData& data = imu.getData();
      Serial.printf("📊 IMU: Accel(%.2f,%.2f,%.2f)g | Gyro(%.1f,%.1f,%.1f)dps | Mag(%.0f,%.0f,%.0f)µT | Temp=%.1f°C\n",
                    data.accel_x, data.accel_y, data.accel_z,
                    data.gyro_x, data.gyro_y, data.gyro_z,
                    data.mag_x, data.mag_y, data.mag_z,
                    data.temperature);
      ahrs.update(data);
      const AHRSData& ahrsData = ahrs.getData();
      if (ahrsData.valid) {
        Serial.printf("📐 AHRS: Roll=%.2f Pitch=%.2f Yaw=%.2f\n", ahrsData.roll, ahrsData.pitch, ahrsData.yaw);
      } else {
        Serial.println("📐 AHRS: Orientation not valid");
      }
      last_imu_print = now;
    }
    return;
  }
  
  if (encoder_test_mode) {
    static uint32_t last_encoder_print = 0;
    if (now - last_encoder_print > 200) {
      Serial.printf("📊 ENCODERS: M1=%d M2=%d M3=%d M4=%d\n",
                    encoder_counts[0], encoder_counts[1], 
                    encoder_counts[2], encoder_counts[3]);
      last_encoder_print = now;
    }
    return;
  }
  
  // ---- Normal Robot Control (X button) ----
  if (js.buttons & PSXBTN_CROSS) {
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
    
    // ---- Get Joystick Commands ----
    float vx = mapStick(js.JoyLeftY, true);   // Forward/backward (inverted)
    float vy = mapStick(js.JoyRightX, false); // Left/right strafe (swapped)
    float wz = mapStick(js.JoyLeftX, false);  // Rotation (swapped)
    
    // ---- Mecanum Wheel Kinematics ----
    // Using corrected geometry for 10.75" square wheelbase with rotation multiplier
    float front_left  =  vx - vy - WHEELBASE_HALF * wz * ROTATION_MULTIPLIER;
    float front_right =  vx + vy + WHEELBASE_HALF * wz * ROTATION_MULTIPLIER;
    float rear_left   =  vx + vy - WHEELBASE_HALF * wz * ROTATION_MULTIPLIER;
    float rear_right  =  vx - vy + WHEELBASE_HALF * wz * ROTATION_MULTIPLIER;
    
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
    
    // ---- Smooth Motor Speed Changes ----
    for (int i = 0; i < 4; i++) {
      motor_speeds[i] = motor_speeds[i] * SPEED_SMOOTH + target_speeds[i] * (1.0f - SPEED_SMOOTH);
    }
    
    // ---- Drive Motors ----
    auto toDuty = [](float speed) { return int(fabsf(speed) * PWM_MAX); };
    auto getDirection = [](float speed) { return (speed > 0) ? 1 : (speed < 0) ? -1 : 0; };
    
  motor1.drive(toDuty(motor_speeds[0]), getDirection(motor_speeds[0]));
  motor2.drive(toDuty(motor_speeds[1]), getDirection(motor_speeds[1]));
  motor3.drive(toDuty(motor_speeds[2]), getDirection(motor_speeds[2]));
  motor4.drive(toDuty(motor_speeds[3]), getDirection(motor_speeds[3]));
    
    // ---- Status Reporting ----
    static uint32_t last_status = 0;
    if (now - last_status > 500) {  // Every 500ms
      Serial.printf("vx=%.2f vy=%.2f wz=%.2f | speed=%.2f | FL=%.2f FR=%.2f RL=%.2f RR=%.2f\n",
                    vx, vy, wz, speed_scale, 
                    motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
      last_status = now;
    }
  } else {
    // Stop motors when X button not pressed
    emergencyStop();
  }
  
  delay(10);  // 100Hz control loop
}
