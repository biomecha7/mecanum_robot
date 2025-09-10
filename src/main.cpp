#include <Arduino.h>
#include <PSX.h>
#include <Wire.h>
#include "MotionController.h"
#include "RobotPins.h"

// ---- Robot Physical Parameters ----
#define WHEELBASE_INCHES 10.75f    // Distance between wheels (inches)
#define WHEEL_DIAMETER_MM 80       // Wheel diameter (mm)
#define WHEELBASE_METERS (WHEELBASE_INCHES * 0.0254f)  // Convert to meters

// ---- Control Parameters ----
static const float WHEELBASE_HALF = WHEELBASE_METERS / 2.0f;  // Corrected geometry
static const float ROTATION_MULTIPLIER = 4.5f;  // Increased rotation sensitivity
static const float DEADBAND = 0.10f;        // Slightly smaller deadband for more responsive control
static const float SPEED_SMOOTH = 0.80f;    // Less smoothing for more responsive feel

// ---- Globals ----
PSX psx;
float speed_scale = 0.70f;  // Start with more conservative speed

// Motion controller
MotionController motionController;

bool controller_connected = false;
uint32_t last_controller_read = 0;

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
  Serial.println("  L1: Slow mode (35%)");
  Serial.println("  R1: Fast mode (100%)");
  Serial.println("  L2: Medium mode (50%)");
  Serial.println("  SELECT: Emergency Stop");
  Serial.println("========================================");

  // Initialize motion controller (handles PWM setup internally)
  Serial.println("Initializing motion controller...");
  motionController.initialize();
  
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
